#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/PidController.hpp"
#include "fase1/Detection.hpp"

class ApproachPoleState : public fsm::State {
public:
    ApproachPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: APPROACH POLE");

        current_pole = *blackboard.get<int>("current_pole");
        
        // Get the target color for current pole
        auto pole_colors = *blackboard.get<std::vector<std::string>>("pole_colors");
        target_color = pole_colors[current_pole];
        
        drone->log("Approaching pole " + std::to_string(current_pole + 1) + " (Color: " + target_color + ")");

        // Get approach parameters
        goal_width = *blackboard.get<float>("goal_width");
        approach_speed = *blackboard.get<float>("approach_speed");
        max_approach_time = *blackboard.get<float>("max_approach_time");
        centering_tolerance = *blackboard.get<float>("centering_tolerance");
        navigation_radius = *blackboard.get<float>("navigation_radius");

        // Initialize PID controllers
        float kp_yaw = *blackboard.get<float>("pid_yaw_kp");
        float ki_yaw = *blackboard.get<float>("pid_yaw_ki");
        float kd_yaw = *blackboard.get<float>("pid_yaw_kd");
        
        float kp_distance = *blackboard.get<float>("pid_distance_kp");
        float ki_distance = *blackboard.get<float>("pid_distance_ki");
        float kd_distance = *blackboard.get<float>("pid_distance_kd");
        
        yaw_pid = std::make_unique<PidController>(kp_yaw, ki_yaw, kd_yaw, 0.5f); // Target center x
        distance_pid = std::make_unique<PidController>(kp_distance, ki_distance, kd_distance, navigation_radius);
        
        start_time = std::chrono::high_resolution_clock::now();
        last_detection_time = start_time;
        approach_complete = false;
        centered = false;
        
        // Get target pole detection from blackboard
        target_detection = *blackboard.get<DronePX4::BoundingBox>("target_pole_detection");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        
        // Check for timeout
        if (elapsed.count() > max_approach_time) {
            drone->log("Approach timeout - proceeding to around pole");
            return "APPROACH_TIMEOUT";
        }

        // Get current post detections
        auto post_detections = drone->getPostDetections();
        
        // Use Detection class to check for target color detection
        Detection detection(post_detections, target_color);
        
        if (!detection.isThereDetection()) {
            // Check if we've been without detection for too long
            auto time_since_last_detection = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_detection_time).count();
            if (time_since_last_detection > 10) {
                drone->log("Lost " + target_color + " pole detection for 10 seconds during approach");
                return "POLE_LOST";
            }
            // Set velocity to 0 if no detection found
            drone->log("No " + target_color + " detection found, stopping");
            drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "";
        } else {
            // Update last detection time since we have a detection
            last_detection_time = current_time;
            
            // Get the closest detection of target color
            DronePX4::BoundingBox current_detection = detection.getClosestBbox();
            
            // Update target detection for next iteration
            target_detection = current_detection;
            blackboard.set<DronePX4::BoundingBox>("target_pole_detection", current_detection);
            
            // Calculate control outputs
            float yaw_correction = yaw_pid->compute(current_detection.center_x);
            
            // Calculate current distance
            float current_distance = goal_width / current_detection.size_x * navigation_radius;
            float forward_velocity = distance_pid->compute(current_distance);
            
            // Check if pole is centered
            float center_error = std::abs(current_detection.center_x - 0.5f);
            centered = (center_error < centering_tolerance);
            
            // Check if we've reached the goal distance (within 10% tolerance)
            bool at_goal_distance = std::abs(current_distance - navigation_radius) < (navigation_radius * 0.1f);
            
            if (centered && at_goal_distance) {
                drone->log("Pole approach complete - centered and at goal distance");
                return "APPROACH_COMPLETE";
            }

            // Get current drone yaw for velocity decomposition
            float current_yaw = drone->getOrientation().z();
            
            // Decompose velocities based on current yaw
            // forward_velocity is in the direction the drone is facing
            float vx = forward_velocity * std::cos(current_yaw); // World frame X velocity
            float vy = forward_velocity * std::sin(current_yaw); // World frame Y velocity
            
            // Apply velocity control with decomposed velocities
            drone->setLocalVelocity(vx, vy, 0.0, yaw_correction);
        }

        return "";
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    std::string target_color;
    float goal_width;
    float approach_speed;
    float max_approach_time;
    float centering_tolerance;
    float navigation_radius;
    
    std::unique_ptr<PidController> yaw_pid;
    std::unique_ptr<PidController> distance_pid;
    
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point last_detection_time;
    bool approach_complete;
    bool centered;
    
    DronePX4::BoundingBox target_detection;
};

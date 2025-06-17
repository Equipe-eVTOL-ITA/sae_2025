#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/PidController.hpp"

class ApproachPoleState : public fsm::State {
public:
    ApproachPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        current_pole = *blackboard.get<int>("current_pole");
        drone->log("STATE: Approaching pole " + std::to_string(current_pole + 1));

        // Get approach parameters
        goal_width = *blackboard.get<float>("goal_width");
        approach_speed = *blackboard.get<float>("approach_speed");
        max_approach_time = *blackboard.get<float>("max_approach_time");
        centering_tolerance = *blackboard.get<float>("centering_tolerance");
        
        // Initialize PID controllers
        float kp_yaw = *blackboard.get<float>("pid_yaw_kp");
        float ki_yaw = *blackboard.get<float>("pid_yaw_ki");
        float kd_yaw = *blackboard.get<float>("pid_yaw_kd");
        
        float kp_width = *blackboard.get<float>("pid_width_kp");
        float ki_width = *blackboard.get<float>("pid_width_ki");
        float kd_width = *blackboard.get<float>("pid_width_kd");
        
        yaw_pid = std::make_unique<PidController>(kp_yaw, ki_yaw, kd_yaw, 0.5f); // Target center x
        width_pid = std::make_unique<PidController>(kp_width, ki_width, kd_width, goal_width);
        
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
        
        if (post_detections.empty()) {
            // Check if we've been without detection for too long
            auto time_since_last_detection = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_detection_time).count();
            if (time_since_last_detection > 10) {
                drone->log("Lost pole detection for 10 seconds during approach");
                return "POLE_LOST";
            }
            // Continue with last known position if recent
            drone->log("No detection found, using last known position");
        } else {
            // Update last detection time since we have a detection
            last_detection_time = current_time;
        }

        // Find the detection that matches our target pole (closest to previous detection)
        DronePX4::BoundingBox current_detection = target_detection; // Use last known if no new detection
        bool found_match = false;
        
        if (!post_detections.empty()) {
            float best_match_distance = std::numeric_limits<float>::max();
            
            for (const auto& detection : post_detections) {
                float distance = std::sqrt(std::pow(detection.center_x - target_detection.center_x, 2) + 
                                         std::pow(detection.center_y - target_detection.center_y, 2));
                
                if (distance < best_match_distance) {
                    best_match_distance = distance;
                    current_detection = detection;
                    found_match = true;
                }
            }
            
            if (found_match) {
                // Update target detection for next iteration
                target_detection = current_detection;
                blackboard.set<DronePX4::BoundingBox>("target_pole_detection", current_detection);
            }
        }

        // Get current drone state
        Eigen::Vector3d current_pos = drone->getLocalPosition();
        float current_yaw = drone->getOrientation().z();

        // Calculate control outputs
        float yaw_correction = yaw_pid->compute(current_detection.center_x);
        float forward_velocity = width_pid->compute(current_detection.size_x);
        
        // Limit velocities
        forward_velocity = std::max(-approach_speed, std::min(approach_speed, forward_velocity));
        yaw_correction = std::max(-0.5f, std::min(0.5f, yaw_correction)); // Limit yaw rate
        
        // Check if pole is centered
        float center_error = std::abs(current_detection.center_x - 0.5f);
        centered = (center_error < centering_tolerance);
        
        // Check if we've reached the goal width
        bool at_goal_width = std::abs(current_detection.size_x - goal_width) < (goal_width * 0.1f); // 10% tolerance
        
        if (centered && at_goal_width) {
            drone->log("Pole approach complete - centered and at goal width");
            return "APPROACH_COMPLETE";
        }

        // Calculate new position
        float new_yaw = current_yaw + yaw_correction;
        
        // Move forward/backward based on width error
        Eigen::Vector3d new_pos = current_pos;
        new_pos.x() += forward_velocity * std::cos(current_yaw) * 0.05f; // 50ms update
        new_pos.y() += forward_velocity * std::sin(current_yaw) * 0.05f;
        
        // Apply control
        drone->setLocalPosition(new_pos.x(), new_pos.y(), new_pos.z(), new_yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting approach pole state");
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    float goal_width;
    float approach_speed;
    float max_approach_time;
    float centering_tolerance;
    
    std::unique_ptr<PidController> yaw_pid;
    std::unique_ptr<PidController> width_pid;
    
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point last_detection_time;
    bool approach_complete;
    bool centered;
    
    DronePX4::BoundingBox target_detection;
};

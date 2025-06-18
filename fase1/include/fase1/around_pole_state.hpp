#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/PidController.hpp"
#include "fase1/Detection.hpp"

class AroundPoleState : public fsm::State {
public:
    AroundPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        // Get current target color
        auto pole_colors = *blackboard.get<std::vector<std::string>>("pole_colors");
        target_color = pole_colors[current_pole];
        
        // Get traversal direction for this pole
        auto pole_directions = *blackboard.get<std::vector<bool>>("pole_directions"); // true = right, false = left
        go_right = pole_directions[current_pole];
        
        drone->log("STATE: Going around pole " + std::to_string(current_pole + 1) + 
                  " (" + target_color + ") to the " + (go_right ? "RIGHT" : "LEFT"));

        // Get navigation parameters
        navigation_radius = *blackboard.get<float>("navigation_radius");
        angular_velocity = *blackboard.get<float>("angular_velocity");
        max_navigation_time = *blackboard.get<float>("max_navigation_time");
        
        // Set angular velocity direction based on traversal direction
        if (go_right) {
            angular_velocity = -std::abs(angular_velocity); // Clockwise (negative)
        } else {
            angular_velocity = std::abs(angular_velocity);  // Counter-clockwise (positive)
        }
        
        // Initialize PID controller for distance control (same parameters as approach_pole_state)
        float kp_distance = *blackboard.get<float>("pid_distance_kp");
        float ki_distance = *blackboard.get<float>("pid_distance_ki");
        float kd_distance = *blackboard.get<float>("pid_distance_kd");
        distance_pid = std::make_unique<PidController>(kp_distance, ki_distance, kd_distance, navigation_radius);
        
        // Calculate total time needed for 210 degrees rotation
        total_time = 7.0f * M_PI / (6.0f * std::abs(angular_velocity));
                
        start_time = std::chrono::high_resolution_clock::now();
        
        drone->log("Navigation: angular_velocity=" + std::to_string(angular_velocity) + 
                  " rad/s, total_time=" + std::to_string(total_time) + 
                  "s, navigation_radius=" + std::to_string(navigation_radius) + "m");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        
        // Check for timeout
        if (elapsed.count() > max_navigation_time) {
            drone->log("Navigation timeout");
            return "NAVIGATION_TIMEOUT";
        }
        
        // Check if rotation is complete based on time
        if (elapsed.count() >= total_time) {
            drone->log("Completed 210° navigation around pole (time-based)");
            // Stop the drone
            drone->setLocalVelocity(0, 0, 0, 0);
            return "NAVIGATION_COMPLETE";
        }

        float current_yaw = drone->getOrientation().z();
        
        // Calculate tangential velocity
        float v_tangent = - navigation_radius * angular_velocity;
        
        // Calculate normal velocity using PID control for distance correction
        float v_normal = 0.0f;
        
        // Update pole position estimate if we have pole detection
        auto post_detections = drone->getPostDetections();
        Detection detection(post_detections, target_color);
        
        if (detection.isThereDetection()) {
            // Refine pole position using detection
            DronePX4::BoundingBox current_detection = detection.getClosestBbox();
            float goal_width = *blackboard.get<float>("goal_width");
            float estimated_distance = goal_width / current_detection.size_x * navigation_radius;
            
            v_normal = distance_pid->compute(estimated_distance);
        }

        drone->log("Yaw: " + std::to_string(current_yaw) +
                  ", vt: " + std::to_string(v_tangent) +
                  ", vn: " + std::to_string(v_normal));
        
        float vx = v_normal * std::cos(current_yaw) - v_tangent * std::sin(current_yaw);
        float vy = v_normal * std::sin(current_yaw) + v_tangent * std::cos(current_yaw);

        drone->log("vx=" + std::to_string(vx) + ", vy=" + std::to_string(vy));
        
        drone->setLocalVelocity(vx, vy, 0, 0.9*angular_velocity);
        
        // Log progress occasionally
        if ((int)(elapsed.count() * 4) % 20 == 0) { // Every 5 seconds
            float progress_percent = elapsed.count() / total_time * 100.0f;
            drone->log("Navigation progress: " + std::to_string(progress_percent) + 
                      "% (" + std::to_string(elapsed.count()) + "s / " + std::to_string(total_time) + "s)");
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        // Increment pole counter
        int next_pole = current_pole + 1;
        blackboard.set<int>("current_pole", next_pole);
        
        drone->log("Completed navigation around pole " + std::to_string(current_pole + 1) + 
                  ". Next pole: " + std::to_string(next_pole + 1));
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    int total_poles;
    bool go_right;
    std::string target_color;
    
    float navigation_radius;
    float angular_velocity;
    float max_navigation_time;
    float total_time; // Time needed for 210° rotation
    
    std::unique_ptr<PidController> distance_pid; // Same PID as approach_pole_state
        
    std::chrono::high_resolution_clock::time_point start_time;
};

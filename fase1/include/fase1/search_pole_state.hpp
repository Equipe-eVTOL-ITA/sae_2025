#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/PidController.hpp"
#include "fase1/Detection.hpp"

class SearchPoleState : public fsm::State {
public:
    SearchPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: SEARCH POLE");

        // Get pole number we're searching for (0-3)
        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        // Get the target color for current pole
        auto pole_colors = *blackboard.get<std::vector<std::string>>("pole_colors");
        target_color = pole_colors[current_pole];
        
        // Get pole direction for current pole (true = right, false = left)
        auto pole_directions = *blackboard.get<std::vector<bool>>("pole_directions");
        go_right = pole_directions[current_pole];

        // Get rotation parameters
        rotation_speed = *blackboard.get<float>("rotation_speed");
        max_search_time = *blackboard.get<float>("max_search_time");
        
        // Set rotation direction based on pole direction
        // If going right around pole: rotate clockwise (negative angular velocity) to find it
        // If going left around pole: rotate counter-clockwise (positive angular velocity) to find it
        if (go_right) {
            angular_velocity = -std::abs(rotation_speed); // Clockwise (negative)
            drone->log("Searching for pole " + std::to_string(current_pole + 1) + "/" + 
                  std::to_string(total_poles) + " (Color: " + target_color + 
                    ") to the right (CLOCKWISE)");
        } else {
            angular_velocity = std::abs(rotation_speed);  // Counter-clockwise (positive)
            drone->log("Searching for pole " + std::to_string(current_pole + 1) + "/" + 
                  std::to_string(total_poles) + " (Color: " + target_color + 
                    ") to the left (ANTI-CLOCKWISE)");
        }
        
        start_time = std::chrono::high_resolution_clock::now();
        pole_found = false;
        last_log_time = -1;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        
        // Check for timeout
        if (elapsed.count() > max_search_time) {
            drone->log("Search timeout for " + target_color + " pole - exceeded " + 
                      std::to_string(max_search_time) + " seconds");
            // Stop rotation
            drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "POLE_NOT_FOUND";
        }

        // Get current post detections
        auto post_detections = drone->getPostDetections();
        
        // Use Detection class to check for target color detection
        Detection detection(post_detections, target_color);
        
        if (detection.isThereDetection()) {
            drone->log("Found " + target_color + " pole! Stopping search rotation.");
            
            // Stop rotation
            drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            
            // Store the detection for the next state
            DronePX4::BoundingBox target_detection = detection.getClosestBbox();
            blackboard.set<DronePX4::BoundingBox>("target_pole_detection", target_detection);
            
            pole_found = true;
            return "POLE_FOUND";
        } else {
            // Continue rotating to search for the pole
            drone->setLocalVelocity(0.0f, 0.0f, 0.0f, angular_velocity);
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    int total_poles;
    std::string target_color;
    bool go_right;
    float rotation_speed;
    float max_search_time;
    float angular_velocity;
    
    std::chrono::high_resolution_clock::time_point start_time;
    bool pole_found{false};
    int last_log_time{-1};
};

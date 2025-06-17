#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/PidController.hpp"

class SearchPoleState : public fsm::State {
public:
    SearchPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        // Get pole number we're searching for (0-3)
        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        drone->log("STATE: Searching for pole " + std::to_string(current_pole + 1) + "/" + std::to_string(total_poles));

        // Get rotation parameters
        rotation_speed = *blackboard.get<float>("rotation_speed");
        max_search_time = *blackboard.get<float>("max_search_time");
        
        // Initialize rotation
        start_time = std::chrono::high_resolution_clock::now();
        initial_yaw = drone->getOrientation().z();
        target_yaw = initial_yaw;
        
        // Get current position
        search_position = drone->getLocalPosition();
        
        pole_found = false;
        search_complete = false;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        
        // Check for timeout
        if (elapsed.count() > max_search_time) {
            drone->log("Search timeout - proceeding to next pole");
            return "POLE_NOT_FOUND";
        }

        // Get post detections
        auto post_detections = drone->getPostDetections();
        
        if (!post_detections.empty()) {
            // Find the most centered detection
            DronePX4::BoundingBox best_detection;
            float best_center_distance = std::numeric_limits<float>::max();
            bool found_detection = false;
            
            for (const auto& detection : post_detections) {
                // Calculate distance from image center
                float center_x = 0.5f; // Image center
                float center_y = 0.5f;
                float distance = std::sqrt(std::pow(detection.center_x - center_x, 2) + 
                                         std::pow(detection.center_y - center_y, 2));
                
                if (distance < best_center_distance) {
                    best_center_distance = distance;
                    best_detection = detection;
                    found_detection = true;
                }
            }
            
            if (found_detection) {
                // Store the target detection in blackboard
                blackboard.set<DronePX4::BoundingBox>("target_pole_detection", best_detection);
                drone->log("Found pole - center distance: " + std::to_string(best_center_distance));
                return "POLE_FOUND";
            }
        }

        // Continue rotating to search for pole
        target_yaw += rotation_speed * 0.05f; // 50ms update rate
        
        // Normalize yaw to [-π, π]
        while (target_yaw > M_PI) target_yaw -= 2 * M_PI;
        while (target_yaw < -M_PI) target_yaw += 2 * M_PI;
        
        // Set position with updated yaw
        drone->setLocalPosition(search_position.x(), search_position.y(), search_position.z(), target_yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting search pole state");
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    int total_poles;
    float rotation_speed;
    float max_search_time;
    
    std::chrono::high_resolution_clock::time_point start_time;
    float initial_yaw;
    float target_yaw;
    Eigen::Vector3d search_position;
    
    bool pole_found;
    bool search_complete;
};

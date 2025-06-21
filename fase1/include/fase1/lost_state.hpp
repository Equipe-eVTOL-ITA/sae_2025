#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/Detection.hpp"

class LostState : public fsm::State {
public:
    LostState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: LOST");

        // Get pole number we're searching for (0-3)
        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        // Get the target color for current pole
        auto pole_colors = *blackboard.get<std::vector<std::string>>("pole_colors");
        target_color = pole_colors[current_pole];
        
        lost_angle = *blackboard.get<float>("lost_angle");
        angular_velocity = *blackboard.get<float>("angular_velocity");
        angular_velocity *= 0.3f;

        search_position = drone->getLocalPosition();
        
        min_yaw = - lost_angle;
        max_yaw = lost_angle;
        
        // Start searching in one direction
        search_direction = 1.0f; // Start searching towards max_yaw
        target_yaw = max_yaw;
        
        start_time = std::chrono::high_resolution_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto post_detections = drone->getPostDetections();        
        Detection detection(post_detections, target_color);
        
        if (detection.isThereDetection()) {
            DronePX4::BoundingBox target_detection = detection.getClosestBbox();
            blackboard.set<DronePX4::BoundingBox>("target_pole_detection", target_detection);
            
            return "POLE_FOUND";
        }

        float current_yaw = drone->getOrientation().z();
        
        float yaw_diff = target_yaw - current_yaw;
        
        if (std::abs(yaw_diff) < 0.1f) {
            // Switch direction and set new target
            search_direction *= -1.0f;
            target_yaw = (search_direction > 0) ? max_yaw : min_yaw;
            
            drone->log("Switching search direction");
        }

        float little_yaw = current_yaw;
        
        if (yaw_diff > angular_velocity) {
            little_yaw = current_yaw + angular_velocity;
        } else if (yaw_diff < -angular_velocity) {
            little_yaw = current_yaw - angular_velocity;
        }
        else{
            little_yaw = target_yaw;
        }

        drone->setLocalPosition(search_position.x(), search_position.y(), 
                               search_position.z(), little_yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        // Stop any movement
        drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    int total_poles;
    std::string target_color;
    float lost_angle;
    
    Eigen::Vector3d search_position;
    float center_yaw;
    float min_yaw;
    float max_yaw;
    float search_direction;
    float target_yaw;
    float angular_velocity;
    
    std::chrono::high_resolution_clock::time_point start_time;
};

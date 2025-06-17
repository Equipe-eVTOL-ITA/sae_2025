#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class ReturnMiddleState : public fsm::State {
public:
    ReturnMiddleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: Returning to middle position");

        // Get return parameters
        return_speed = *blackboard.get<float>("return_speed");
        position_tolerance = *blackboard.get<float>("position_tolerance");
        max_return_time = *blackboard.get<float>("max_return_time");
        
        // Get middle position (starting position)
        middle_position = *blackboard.get<Eigen::Vector3d>("middle_position");
        
        // Get current position
        start_position = drone->getLocalPosition();
        current_position = start_position;
        
        // Calculate distance to middle
        distance_to_middle = (middle_position - start_position).norm();
        
        start_time = std::chrono::high_resolution_clock::now();
        return_complete = false;
        
        drone->log("Return: distance to middle = " + std::to_string(distance_to_middle) + "m");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        float dt = 0.05f; // 50ms update rate
        
        // Check for timeout
        if (elapsed.count() > max_return_time) {
            drone->log("Return timeout - proceeding to landing");
            return "RETURN_TIMEOUT";
        }

        // Get current position
        current_position = drone->getLocalPosition();
        
        // Calculate vector to middle position
        Eigen::Vector3d to_middle = middle_position - current_position;
        float distance_remaining = to_middle.norm();
        
        // Check if we've reached the middle position
        if (distance_remaining < position_tolerance) {
            drone->log("Reached middle position");
            return "RETURN_COMPLETE";
        }
        
        // Calculate movement direction (unit vector towards middle)
        Eigen::Vector3d direction = to_middle.normalized();
        
        // Calculate movement step
        float step_size = std::min(return_speed * dt, distance_remaining);
        Eigen::Vector3d movement = direction * step_size;
        
        // Calculate new position
        Eigen::Vector3d new_position = current_position + movement;
        
        // Calculate yaw to face the middle position
        float target_yaw = std::atan2(to_middle.y(), to_middle.x());
        
        // Apply new position and orientation
        drone->setLocalPosition(new_position.x(), new_position.y(), new_position.z(), target_yaw);
        
        // Log progress occasionally
        if ((int)(elapsed.count() * 4) % 20 == 0) { // Every 5 seconds
            drone->log("Return progress: " + std::to_string(distance_remaining) + "m remaining");
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Completed return to middle");
    }

private:
    Drone* drone{nullptr};
    float return_speed;
    float position_tolerance;
    float max_return_time;
    
    Eigen::Vector3d middle_position;
    Eigen::Vector3d start_position;
    Eigen::Vector3d current_position;
    
    float distance_to_middle;
    
    std::chrono::high_resolution_clock::time_point start_time;
    bool return_complete;
};

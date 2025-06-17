#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class AroundPoleState : public fsm::State {
public:
    AroundPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        // Get traversal direction for this pole
        auto pole_directions = *blackboard.get<std::vector<bool>>("pole_directions"); // true = right, false = left
        go_right = pole_directions[current_pole];
        
        drone->log("STATE: Going around pole " + std::to_string(current_pole + 1) + 
                  " to the " + (go_right ? "RIGHT" : "LEFT"));

        // Get navigation parameters
        radius = *blackboard.get<float>("navigation_radius");
        angular_velocity = *blackboard.get<float>("angular_velocity");
        max_navigation_time = *blackboard.get<float>("max_navigation_time");
        
        // Get current position and calculate pole position
        current_pos = drone->getLocalPosition();
        current_yaw = drone->getOrientation().z();
        
        // Estimate pole position from current drone position and yaw
        // Assume we're at the correct distance from pole when we start
        pole_position.x() = current_pos.x() + radius * std::cos(current_yaw);
        pole_position.y() = current_pos.y() + radius * std::sin(current_yaw);
        pole_position.z() = current_pos.z(); // Same altitude as drone
        
        // Calculate initial angle around pole
        Eigen::Vector2d to_drone = Eigen::Vector2d(current_pos.x() - pole_position.x(), 
                                                   current_pos.y() - pole_position.y());
        initial_angle = std::atan2(to_drone.y(), to_drone.x());
        current_angle = initial_angle;
        
        // Determine target angle (π radians = 180 degrees around pole)
        if (go_right) {
            target_angle = initial_angle - M_PI; // Clockwise
            angular_velocity = -std::abs(angular_velocity);
        } else {
            target_angle = initial_angle + M_PI; // Counter-clockwise
            angular_velocity = std::abs(angular_velocity);
        }
        
        // Normalize target angle
        while (target_angle > M_PI) target_angle -= 2 * M_PI;
        while (target_angle < -M_PI) target_angle += 2 * M_PI;
        
        start_time = std::chrono::high_resolution_clock::now();
        navigation_complete = false;
        
        drone->log("Navigation: initial_angle=" + std::to_string(initial_angle) + 
                  " target_angle=" + std::to_string(target_angle) + 
                  " radius=" + std::to_string(radius));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        float dt = 0.05f; // 50ms update rate
        
        // Check for timeout
        if (elapsed.count() > max_navigation_time) {
            drone->log("Navigation timeout");
            return "NAVIGATION_TIMEOUT";
        }

        // Update current angle
        current_angle += angular_velocity * dt;
        
        // Check if we've completed the semicircle
        float angle_progress;
        if (go_right) {
            angle_progress = initial_angle - current_angle;
        } else {
            angle_progress = current_angle - initial_angle;
        }
        
        // Normalize angle progress
        while (angle_progress > M_PI) angle_progress -= 2 * M_PI;
        while (angle_progress < -M_PI) angle_progress += 2 * M_PI;
        
        // Check if we've completed approximately π radians (180 degrees)
        if (std::abs(angle_progress) >= M_PI * 0.9f) { // 90% of π to account for some error
            drone->log("Completed semicircle around pole");
            return "NAVIGATION_COMPLETE";
        }
        
        // Calculate new position on circle
        Eigen::Vector3d new_pos;
        new_pos.x() = pole_position.x() + radius * std::cos(current_angle);
        new_pos.y() = pole_position.y() + radius * std::sin(current_angle);
        new_pos.z() = current_pos.z(); // Maintain altitude
        
        // Calculate yaw to face tangent to circle (forward direction of movement)
        float tangent_yaw = current_angle + (go_right ? -M_PI/2 : M_PI/2);
        
        // Normalize yaw
        while (tangent_yaw > M_PI) tangent_yaw -= 2 * M_PI;
        while (tangent_yaw < -M_PI) tangent_yaw += 2 * M_PI;
        
        // Apply new position and orientation
        drone->setLocalPosition(new_pos.x(), new_pos.y(), new_pos.z(), tangent_yaw);
        
        // Update current position for next iteration
        current_pos = new_pos;
        current_yaw = tangent_yaw;
        
        // Log progress occasionally
        if ((int)(elapsed.count() * 4) % 20 == 0) { // Every 5 seconds
            drone->log("Navigation progress: " + std::to_string(std::abs(angle_progress) * 180 / M_PI) + " degrees");
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
    
    float radius;
    float angular_velocity;
    float max_navigation_time;
    
    Eigen::Vector3d current_pos;
    Eigen::Vector3d pole_position;
    float current_yaw;
    
    float initial_angle;
    float current_angle;
    float target_angle;
    
    std::chrono::high_resolution_clock::time_point start_time;
    bool navigation_complete;
};

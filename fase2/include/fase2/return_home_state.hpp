#pragma once

#include "fsm/state.hpp"
#include "drone/Drone.hpp"
#include "fase2/PidController.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>

class ReturnHomeState : public fsm::State {
private:
    Drone* drone;
    
    // Home position
    Eigen::Vector3d home_position_;
    double yaw_;
    
    // Control parameters
    double max_velocity_;
    double position_tolerance_;
    double return_height_;
    double navigation_timeout_;
    
    // Navigation timing
    std::chrono::steady_clock::time_point navigation_start_time_;
    
    bool isAtHome() {
        Eigen::Vector3d current_pos = drone->getLocalPosition();
        Eigen::Vector3d horizontal_diff = current_pos - home_position_;
        horizontal_diff.z() = 0;  // Ignore vertical difference for now
        
        return horizontal_diff.norm() < position_tolerance_;
    }

public:
    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");

        drone->log("STATE: RETURN HOME");
        
        // Get parameters from blackboard
        max_velocity_ = *blackboard.get<double>("max_horizontal_velocity");
        position_tolerance_ = *blackboard.get<double>("home_position_tolerance");
        return_height_ = *blackboard.get<double>("return_height");
        navigation_timeout_ = *blackboard.get<double>("return_navigation_timeout");
        
        // Get home position (stored during takeoff)
        home_position_ = Eigen::Vector3d({0.0, 0.0, return_height_});
        yaw_ = drone->getOrientation()[2];
        
        navigation_start_time_ = std::chrono::steady_clock::now();
        
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Check for navigation timeout
        auto navigation_elapsed = std::chrono::steady_clock::now() - navigation_start_time_;
        if (std::chrono::duration_cast<std::chrono::seconds>(navigation_elapsed).count() > navigation_timeout_) {
            drone->log("Return navigation timeout");
            return "TIMEOUT";
        }
        
        // Check if we've reached home
        if (isAtHome()) {
            drone->log("Arrived at home position");
            return "ARRIVED_HOME";
        }
        
        // Navigate to home position using position control
        Eigen::Vector3d target_position = home_position_;
        target_position.z() = return_height_;  // Maintain return height
        
        Eigen::Vector3d current_pos = drone->getLocalPosition();
        Eigen::Vector3d position_error = target_position - current_pos;
        
        // Limit velocity to max_velocity_
        if (position_error.norm() > max_velocity_) {
            position_error = position_error.normalized() * max_velocity_;
        }
        
        Eigen::Vector3d target_vel_position = current_pos + position_error;
        
        // Use position control to navigate home
        drone->setLocalPosition(target_vel_position.x(), target_vel_position.y(), target_vel_position.z(), yaw_);
        
        // Periodic logging
        static int log_counter = 0;
        if (++log_counter % 40 == 0) {  // Log every 2 seconds at 20Hz
            double distance_to_home = (current_pos - home_position_).norm();
            drone->log("Returning home - distance: " + std::to_string(distance_to_home) + "m");
        }
        
        return "";  // Continue in this state
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Arrived at home position - ready for precision alignment");
    }
};

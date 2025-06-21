#pragma once

#include "fsm/state.hpp"
#include "drone/Drone.hpp"
#include "fase2/PidController.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>

class GoToHoseState : public fsm::State {
private:
    Drone* drone;
    
    // Image-based PID controllers following approach_pole_state pattern
    std::unique_ptr<PidController> lateral_pid;         // Lateral position control (target center x = 0.5)
    std::unique_ptr<PidController> height_pid;          // Height control (target center y = 0.5)
    std::unique_ptr<PidController> yaw_pid;             // Yaw control (target direction = 0.0 degrees)
    
    // Subscriptions for line detection
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr line_centroid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr line_direction_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr hose_position_sub_;
    
    // Line following state
    struct LineState {
        bool has_line_detection = false;
        double centroid_x = 0.5;        // Normalized line centroid x (0-1)
        double centroid_y = 0.5;        // Normalized line centroid y (0-1)
        double direction_angle = 0.0;   // Line direction in degrees
        double confidence = 0.0;        // Line detection confidence
        std::chrono::steady_clock::time_point last_detection;
    } line_state_;
    
    // Hose detection state
    struct HoseState {
        bool has_hose_detection = false;
        double position_x = 0.5;        // Normalized hose position x (0-1)
        double position_y = 0.5;        // Normalized hose position y (0-1)
        double confidence = 0.0;        // Hose detection confidence
        std::chrono::steady_clock::time_point last_detection;
    } hose_state_;
    
    // Control parameters
    double forward_velocity_;
    double max_lateral_velocity_;
    double max_yaw_rate_;
    double target_height_;
    double line_confidence_threshold_;
    double hose_confidence_threshold_;
    double hose_detection_timeout_;
    
    // Mission timing
    std::chrono::steady_clock::time_point mission_start_time_;
    double max_mission_time_;
    
    void setupSubscriptions() {
        // Line detection is now handled by Drone class interface
        // We'll use the new Drone interface methods:
        // - drone->getLineDetection()
        // - drone->getHoseDetection()
        // - drone->isLineDetectionRecent()
        // - drone->isHoseDetectionRecent()
        
        // No need to create subscriptions here - they're managed by Drone class
    }
    
    bool isLineDetectionValid() {
        return drone->isLineDetectionRecent(0.5);  // 500ms timeout
    }
    
    bool isHoseDetectionValid() {
        return drone->isHoseDetectionRecent(1.0);  // 1000ms timeout
    }

public:
    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        
        drone->log("STATE: GO TO HOSE");
        
        // Get parameters from blackboard
        forward_velocity_ = *blackboard.get<double>("forward_velocity");
        max_lateral_velocity_ = *blackboard.get<double>("max_lateral_velocity");
        max_yaw_rate_ = *blackboard.get<double>("max_yaw_rate");
        target_height_ = *blackboard.get<double>("line_following_height");
        line_confidence_threshold_ = *blackboard.get<double>("line_confidence_threshold");
        hose_confidence_threshold_ = *blackboard.get<double>("hose_confidence_threshold");
        hose_detection_timeout_ = *blackboard.get<double>("hose_detection_timeout");
        max_mission_time_ = *blackboard.get<double>("max_line_following_time");
        
        // Initialize PID controllers with 6 parameters (3 translation + 3 yaw)
        float kp_lateral = *blackboard.get<float>("line_follow_lateral_kp");
        float ki_lateral = *blackboard.get<float>("line_follow_lateral_ki");
        float kd_lateral = *blackboard.get<float>("line_follow_lateral_kd");
        
        float kp_height = *blackboard.get<float>("line_follow_height_kp");
        float ki_height = *blackboard.get<float>("line_follow_height_ki");
        float kd_height = *blackboard.get<float>("line_follow_height_kd");
        
        float kp_yaw = *blackboard.get<float>("line_follow_yaw_kp");
        float ki_yaw = *blackboard.get<float>("line_follow_yaw_ki");
        float kd_yaw = *blackboard.get<float>("line_follow_yaw_kd");
        
        lateral_pid = std::make_unique<PidController>(kp_lateral, ki_lateral, kd_lateral, 0.5f); // Target center x
        height_pid = std::make_unique<PidController>(kp_height, ki_height, kd_height, 0.5f);    // Target center y
        yaw_pid = std::make_unique<PidController>(kp_yaw, ki_yaw, kd_yaw, 0.0f);               // Target direction
        
        mission_start_time_ = std::chrono::steady_clock::now();
        
        drone->log("Line following PID initialized - following line to hose");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Check for mission timeout
        auto mission_elapsed = std::chrono::steady_clock::now() - mission_start_time_;
        if (std::chrono::duration_cast<std::chrono::seconds>(mission_elapsed).count() > max_mission_time_) {
            drone->log("Line following mission timeout");
            return "TIMEOUT";
        }
        
        // Check for hose detection first (mission completion)
        if (isHoseDetectionValid()) {
            auto time_since_first_hose = std::chrono::steady_clock::now() - hose_state_.last_detection;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(time_since_first_hose).count() > hose_detection_timeout_) {
                drone->log("Hose detected consistently - transitioning to alignment");
                return "HOSE_DETECTED";
            }
        }
        
        // Continue line following
        if (isLineDetectionValid()) {
            // Get line detection data from Drone interface
            auto line_data = drone->getLineDetection();
            
            // Image-based PID control following approach_pole_state pattern
            // Calculate lateral velocity based on line centroid (target center x = 0.5)
            double lateral_velocity = lateral_pid->compute(line_data.centroid_x);
            lateral_velocity = std::clamp(lateral_velocity, -max_lateral_velocity_, max_lateral_velocity_);
            
            // Calculate height velocity based on line centroid y (target center y = 0.5)
            double height_velocity = height_pid->compute(line_data.centroid_y);
            height_velocity = std::clamp(height_velocity, -0.5, 0.5);
            
            // Calculate yaw rate based on line direction (target direction = 0.0 degrees)
            double yaw_rate = yaw_pid->compute(line_data.direction_angle);
            yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
            
            // Apply velocity control - image-based control
            // lateral_velocity controls side-to-side movement to keep line centered
            // height_velocity adjusts altitude to maintain proper line detection
            // forward_velocity is constant for line following progress
            drone->setLocalVelocity(forward_velocity_, lateral_velocity, height_velocity, yaw_rate);
            
            // Periodic logging
            static int log_counter = 0;
            if (++log_counter % 20 == 0) {  // Log every second at 20Hz
                drone->log("Following line: centroid_x=" + std::to_string(line_data.centroid_x) + 
                          ", centroid_y=" + std::to_string(line_data.centroid_y) +
                          ", direction=" + std::to_string(line_data.direction_angle));
            }
        } else {
            // No valid line detection - hover and search
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            drone->log("No valid line detection - hovering");
        }
        
        return "";  // Continue in this state
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Stop motion
        drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        
        // Store hose position for next state
        if (isHoseDetectionValid()) {
            auto hose_data = drone->getHoseDetection();
            blackboard.set<double>("hose_position_x", hose_data.position_x);
            blackboard.set<double>("hose_position_y", hose_data.position_y);
        }
        
        drone->log("Completed line following - hose detected");
    }
};

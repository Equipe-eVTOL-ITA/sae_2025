#pragma once

#include "fsm/state.hpp"
#include "drone/Drone.hpp"
#include "fase2/Detection.hpp"
#include "fase2/PidController.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>

class AlignHomeState : public fsm::State {
private:
    Drone* drone;
    
    // Image-based PID controllers following approach_pole_state pattern
    std::unique_ptr<PidController> lateral_pid;         // Lateral position control (target center x = 0.5)
    std::unique_ptr<PidController> vertical_pid;        // Vertical position control (target center y = 0.5)
    std::unique_ptr<PidController> distance_pid;        // Distance control based on detection size
    std::unique_ptr<PidController> yaw_pid;             // Yaw control for stable alignment
    
    // Landing base detection state
    struct LandingBaseState {
        bool has_detection = false;
        DronePX4::BoundingBox closest_detection;
        std::chrono::steady_clock::time_point last_detection;
        
        // Stability tracking
        std::vector<double> recent_x_positions;
        std::vector<double> recent_y_positions;
        std::vector<double> recent_sizes;
        static const size_t STABILITY_BUFFER_SIZE = 15;
    } base_state_;
    
    // Control parameters
    double alignment_tolerance_;
    double max_velocity_;
    double stability_threshold_;
    double alignment_timeout_;
    double target_landing_height_;
    double target_detection_size_;
    
    // Alignment timing
    std::chrono::steady_clock::time_point alignment_start_time_;
    std::chrono::steady_clock::time_point stable_alignment_start_;
    double stable_hold_duration_;
    bool alignment_stable_ = false;
    
    bool isDetectionValid() {
        auto detections = drone->getVerticalBboxes();
        
        if (detections.empty()) {
            base_state_.has_detection = false;
            return false;
        }
        
        // Find the closest detection to image center
        Eigen::Vector2d image_center(0.5, 0.5);
        double min_distance = std::numeric_limits<double>::max();
        DronePX4::BoundingBox closest_detection;
        
        for (const auto& detection : detections) {
            // Filter by class if needed (landing base detector should only detect landing bases)
            Eigen::Vector2d detection_center(detection.center_x, detection.center_y);
            double distance = (detection_center - image_center).norm();
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_detection = detection;
            }
        }
        
        base_state_.closest_detection = closest_detection;
        base_state_.has_detection = true;
        base_state_.last_detection = std::chrono::steady_clock::now();
        
        // Update stability buffer
        base_state_.recent_x_positions.push_back(closest_detection.center_x);
        base_state_.recent_y_positions.push_back(closest_detection.center_y);
        base_state_.recent_sizes.push_back(closest_detection.size_x * closest_detection.size_y);
        
        if (base_state_.recent_x_positions.size() > LandingBaseState::STABILITY_BUFFER_SIZE) {
            base_state_.recent_x_positions.erase(base_state_.recent_x_positions.begin());
            base_state_.recent_y_positions.erase(base_state_.recent_y_positions.begin());
            base_state_.recent_sizes.erase(base_state_.recent_sizes.begin());
        }
        
        return true;
    }
    
    bool isAlignmentStable() {
        if (base_state_.recent_x_positions.size() < LandingBaseState::STABILITY_BUFFER_SIZE) {
            return false;
        }
        
        // Calculate standard deviation of recent positions
        double sum_x = 0, sum_y = 0, sum_size = 0;
        for (size_t i = 0; i < base_state_.recent_x_positions.size(); ++i) {
            sum_x += base_state_.recent_x_positions[i];
            sum_y += base_state_.recent_y_positions[i];
            sum_size += base_state_.recent_sizes[i];
        }
        double mean_x = sum_x / base_state_.recent_x_positions.size();
        double mean_y = sum_y / base_state_.recent_y_positions.size();
        double mean_size = sum_size / base_state_.recent_sizes.size();
        
        double var_x = 0, var_y = 0, var_size = 0;
        for (size_t i = 0; i < base_state_.recent_x_positions.size(); ++i) {
            var_x += (base_state_.recent_x_positions[i] - mean_x) * (base_state_.recent_x_positions[i] - mean_x);
            var_y += (base_state_.recent_y_positions[i] - mean_y) * (base_state_.recent_y_positions[i] - mean_y);
            var_size += (base_state_.recent_sizes[i] - mean_size) * (base_state_.recent_sizes[i] - mean_size);
        }
        double std_x = sqrt(var_x / base_state_.recent_x_positions.size());
        double std_y = sqrt(var_y / base_state_.recent_y_positions.size());
        double std_size = sqrt(var_size / base_state_.recent_sizes.size());
        
        // Check if position is stable and close to center
        bool position_stable = (std_x < stability_threshold_) && (std_y < stability_threshold_);
        bool size_stable = std_size < stability_threshold_;
        bool position_centered = (abs(mean_x - 0.5) < alignment_tolerance_) && (abs(mean_y - 0.5) < alignment_tolerance_);
        bool size_appropriate = abs(mean_size - target_detection_size_) < 0.1;  // Within 10% of target size
        
        return position_stable && size_stable && position_centered && size_appropriate;
    }

public:
    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        
        drone->log("STATE: ALIGN HOME");
        
        // Get parameters from blackboard
        alignment_tolerance_ = *blackboard.get<double>("home_alignment_tolerance");
        max_velocity_ = *blackboard.get<double>("home_alignment_velocity");
        stability_threshold_ = *blackboard.get<double>("home_stability_threshold");
        alignment_timeout_ = *blackboard.get<double>("home_alignment_timeout");
        stable_hold_duration_ = *blackboard.get<double>("home_stable_hold_time");
        target_landing_height_ = *blackboard.get<double>("target_landing_height");
        target_detection_size_ = *blackboard.get<double>("target_detection_size");
        
        // Initialize PID controllers with 6 parameters (3 translation + 3 yaw)
        float kp_lateral = *blackboard.get<float>("home_align_lateral_kp");
        float ki_lateral = *blackboard.get<float>("home_align_lateral_ki");
        float kd_lateral = *blackboard.get<float>("home_align_lateral_kd");
        
        float kp_vertical = *blackboard.get<float>("home_align_vertical_kp");
        float ki_vertical = *blackboard.get<float>("home_align_vertical_ki");
        float kd_vertical = *blackboard.get<float>("home_align_vertical_kd");
        
        float kp_distance = *blackboard.get<float>("home_align_distance_kp");
        float ki_distance = *blackboard.get<float>("home_align_distance_ki");
        float kd_distance = *blackboard.get<float>("home_align_distance_kd");
        
        float kp_yaw = *blackboard.get<float>("home_align_yaw_kp");
        float ki_yaw = *blackboard.get<float>("home_align_yaw_ki");
        float kd_yaw = *blackboard.get<float>("home_align_yaw_kd");
        
        lateral_pid = std::make_unique<PidController>(kp_lateral, ki_lateral, kd_lateral, 0.5f);        // Target center x
        vertical_pid = std::make_unique<PidController>(kp_vertical, ki_vertical, kd_vertical, 0.5f);    // Target center y
        distance_pid = std::make_unique<PidController>(kp_distance, ki_distance, kd_distance, target_detection_size_); // Target size
        yaw_pid = std::make_unique<PidController>(kp_yaw, ki_yaw, kd_yaw, 0.0f);                       // Stable yaw
        
        alignment_start_time_ = std::chrono::steady_clock::now();
        alignment_stable_ = false;
        
        drone->log("Home alignment PID initialized - precision landing positioning");
        
        // Clear stability buffer
        base_state_.recent_x_positions.clear();
        base_state_.recent_y_positions.clear();
        base_state_.recent_sizes.clear();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Check for alignment timeout
        auto alignment_elapsed = std::chrono::steady_clock::now() - alignment_start_time_;
        if (std::chrono::duration_cast<std::chrono::seconds>(alignment_elapsed).count() > alignment_timeout_) {
            drone->log("Home alignment timeout");
            return "TIMEOUT";
        }
        
        if (isDetectionValid()) {
            // Image-based PID control following approach_pole_state pattern
            // Calculate current detection size
            double current_size = base_state_.closest_detection.size_x * base_state_.closest_detection.size_y;
            
            // Calculate velocities using PID controllers (target center = 0.5)
            double lateral_velocity = lateral_pid->compute(base_state_.closest_detection.center_x);
            double vertical_velocity = vertical_pid->compute(base_state_.closest_detection.center_y);
            double height_velocity = distance_pid->compute(current_size);
            double yaw_rate = yaw_pid->compute(0.0);  // Maintain stable yaw
            
            // Clamp velocities
            lateral_velocity = std::clamp(lateral_velocity, -max_velocity_, max_velocity_);
            vertical_velocity = std::clamp(vertical_velocity, -max_velocity_, max_velocity_);
            height_velocity = std::clamp(height_velocity, -0.3, 0.3);  // Slower height adjustments
            
            // Apply velocity control for precision alignment
            drone->setLocalVelocity(lateral_velocity, vertical_velocity, height_velocity, yaw_rate);
            
            // Check alignment stability
            if (isAlignmentStable()) {
                if (!alignment_stable_) {
                    // Just became stable
                    alignment_stable_ = true;
                    stable_alignment_start_ = std::chrono::steady_clock::now();
                    drone->log("Home base alignment achieved - starting stable hold");
                } else {
                    // Check if stable hold duration is met
                    auto stable_elapsed = std::chrono::steady_clock::now() - stable_alignment_start_;
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(stable_elapsed).count() > stable_hold_duration_ * 1000) {
                        drone->log("Stable home alignment completed - ready for landing");
                        return "ALIGNED_HOME";
                    }
                }
            } else {
                // Lost stability
                alignment_stable_ = false;
            }
            
            // Periodic logging
            static int log_counter = 0;
            if (++log_counter % 20 == 0) {  // Log every second at 20Hz
                double current_size = base_state_.closest_detection.size_x * base_state_.closest_detection.size_y;
                drone->log("Aligning with home: center_x=" + std::to_string(base_state_.closest_detection.center_x) + 
                          ", center_y=" + std::to_string(base_state_.closest_detection.center_y) +
                          ", size=" + std::to_string(current_size) +
                          ", stable=" + (alignment_stable_ ? "true" : "false"));
            }
        } else {
            // No valid base detection - hover and search
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            
            static int no_detection_counter = 0;
            if (++no_detection_counter % 40 == 0) {  // Log every 2 seconds
                drone->log("No valid landing base detection during alignment");
            }
        }
        
        return "";  // Continue in this state
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Stop motion
        drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        
        drone->log("Home base alignment completed - ready for final landing");
    }
};

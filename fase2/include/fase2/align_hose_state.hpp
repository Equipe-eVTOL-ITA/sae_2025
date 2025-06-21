#pragma once

#include "fsm/state.hpp"
#include "drone/Drone.hpp"
#include "fase2/PidController.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>

class AlignHoseState : public fsm::State {
private:
    Drone* drone;
    
    // Image-based PID controllers following approach_pole_state pattern
    std::unique_ptr<PidController> lateral_pid;         // Lateral position control (target center x = 0.5)
    std::unique_ptr<PidController> vertical_pid;        // Vertical position control (target center y = 0.5)
    std::unique_ptr<PidController> yaw_pid;             // Yaw control for stable alignment
    
    // Hose detection state
    struct HoseState {
        bool has_detection = false;
        double position_x = 0.5;        // Normalized hose position x (0-1)
        double position_y = 0.5;        // Normalized hose position y (0-1)
        double confidence = 0.0;        // Hose detection confidence
        std::chrono::steady_clock::time_point last_detection;
        
        // Stability tracking
        std::vector<double> recent_x_positions;
        std::vector<double> recent_y_positions;
        static const size_t STABILITY_BUFFER_SIZE = 10;
    } hose_state_;
    
    // Control parameters
    double alignment_tolerance_;
    double confidence_threshold_;
    double max_velocity_;
    double stability_threshold_;
    double alignment_timeout_;
    double payload_drop_height_;
    
    // Alignment timing
    std::chrono::steady_clock::time_point alignment_start_time_;
    std::chrono::steady_clock::time_point stable_alignment_start_;
    double stable_hold_duration_;
    bool alignment_stable_ = false;
    
    
    void setupSubscription() {
        // Hose detection is now handled by Drone class interface
        // We'll use the new Drone interface methods:
        // - drone->getHoseDetection()
        // - drone->isHoseDetectionRecent()
    }
    
    bool isHoseDetectionValid() {
        return drone->isHoseDetectionRecent(0.5);  // 500ms timeout
    }
    
    bool isAlignmentStable() {
        if (hose_state_.recent_x_positions.size() < HoseState::STABILITY_BUFFER_SIZE) {
            return false;
        }
        
        // Calculate standard deviation of recent positions
        double sum_x = 0, sum_y = 0;
        for (size_t i = 0; i < hose_state_.recent_x_positions.size(); ++i) {
            sum_x += hose_state_.recent_x_positions[i];
            sum_y += hose_state_.recent_y_positions[i];
        }
        double mean_x = sum_x / hose_state_.recent_x_positions.size();
        double mean_y = sum_y / hose_state_.recent_y_positions.size();
        
        double var_x = 0, var_y = 0;
        for (size_t i = 0; i < hose_state_.recent_x_positions.size(); ++i) {
            var_x += (hose_state_.recent_x_positions[i] - mean_x) * (hose_state_.recent_x_positions[i] - mean_x);
            var_y += (hose_state_.recent_y_positions[i] - mean_y) * (hose_state_.recent_y_positions[i] - mean_y);
        }
        double std_x = sqrt(var_x / hose_state_.recent_x_positions.size());
        double std_y = sqrt(var_y / hose_state_.recent_y_positions.size());
        
        // Check if position is stable and close to center
        bool position_stable = (std_x < stability_threshold_) && (std_y < stability_threshold_);
        bool position_centered = (abs(mean_x - 0.5) < alignment_tolerance_) && (abs(mean_y - 0.5) < alignment_tolerance_);
        
        return position_stable && position_centered;
    }

public:
    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        
        drone->log("STATE: ALIGN HOSE");
        
        // Get parameters from blackboard
        alignment_tolerance_ = *blackboard.get<double>("hose_alignment_tolerance");
        confidence_threshold_ = *blackboard.get<double>("hose_confidence_threshold");
        max_velocity_ = *blackboard.get<double>("hose_alignment_velocity");
        stability_threshold_ = *blackboard.get<double>("hose_stability_threshold");
        alignment_timeout_ = *blackboard.get<double>("hose_alignment_timeout");
        stable_hold_duration_ = *blackboard.get<double>("hose_stable_hold_time");
        payload_drop_height_ = *blackboard.get<double>("payload_drop_height");
        
        // Initialize PID controllers with 6 parameters (3 translation + 3 yaw)
        float kp_lateral = *blackboard.get<float>("hose_align_lateral_kp");
        float ki_lateral = *blackboard.get<float>("hose_align_lateral_ki");
        float kd_lateral = *blackboard.get<float>("hose_align_lateral_kd");
        
        float kp_vertical = *blackboard.get<float>("hose_align_vertical_kp");
        float ki_vertical = *blackboard.get<float>("hose_align_vertical_ki");
        float kd_vertical = *blackboard.get<float>("hose_align_vertical_kd");
        
        float kp_yaw = *blackboard.get<float>("hose_align_yaw_kp");
        float ki_yaw = *blackboard.get<float>("hose_align_yaw_ki");
        float kd_yaw = *blackboard.get<float>("hose_align_yaw_kd");
        
        lateral_pid = std::make_unique<PidController>(kp_lateral, ki_lateral, kd_lateral, 0.5f);    // Target center x
        vertical_pid = std::make_unique<PidController>(kp_vertical, ki_vertical, kd_vertical, 0.5f); // Target center y
        yaw_pid = std::make_unique<PidController>(kp_yaw, ki_yaw, kd_yaw, 0.0f);                   // Stable yaw
        
        alignment_start_time_ = std::chrono::steady_clock::now();
        alignment_stable_ = false;
        
        // Initialize with stored hose position if available
        if (blackboard.contains("hose_position_x")) {
            hose_state_.position_x = *blackboard.get<double>("hose_position_x");
            hose_state_.position_y = *blackboard.get<double>("hose_position_y");
        }
        
        drone->log("Hose alignment PID initialized - precision positioning");
        
        // Move to payload drop height
        Eigen::Vector3d current_pos = drone->getLocalPosition();
        drone->setLocalPosition(current_pos.x(), current_pos.y(), payload_drop_height_, 0.0);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Check for alignment timeout
        auto alignment_elapsed = std::chrono::steady_clock::now() - alignment_start_time_;
        if (std::chrono::duration_cast<std::chrono::seconds>(alignment_elapsed).count() > alignment_timeout_) {
            drone->log("Hose alignment timeout");
            return "TIMEOUT";
        }
        
        if (isHoseDetectionValid()) {
            // Get hose detection data from Drone interface
            auto hose_data = drone->getHoseDetection();
            
            // Update stability buffer
            hose_state_.recent_x_positions.push_back(hose_data.position_x);
            hose_state_.recent_y_positions.push_back(hose_data.position_y);
            
            if (hose_state_.recent_x_positions.size() > HoseState::STABILITY_BUFFER_SIZE) {
                hose_state_.recent_x_positions.erase(hose_state_.recent_x_positions.begin());
                hose_state_.recent_y_positions.erase(hose_state_.recent_y_positions.begin());
            }
            
            // Image-based PID control following approach_pole_state pattern
            // Calculate velocities using PID controllers (target center = 0.5)
            double lateral_velocity = lateral_pid->compute(hose_data.position_x);
            double vertical_velocity = vertical_pid->compute(hose_data.position_y);
            double yaw_rate = yaw_pid->compute(0.0);  // Maintain stable yaw
            
            // Clamp velocities
            lateral_velocity = std::clamp(lateral_velocity, -max_velocity_, max_velocity_);
            vertical_velocity = std::clamp(vertical_velocity, -max_velocity_, max_velocity_);
            
            // Apply velocity control for precision alignment
            drone->setLocalVelocity(lateral_velocity, vertical_velocity, 0.0, yaw_rate);
            
            // Check alignment stability
            if (isAlignmentStable()) {
                if (!alignment_stable_) {
                    // Just became stable
                    alignment_stable_ = true;
                    stable_alignment_start_ = std::chrono::steady_clock::now();
                    drone->log("Hose alignment achieved - starting stable hold");
                } else {
                    // Check if stable hold duration is met
                    auto stable_elapsed = std::chrono::steady_clock::now() - stable_alignment_start_;
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(stable_elapsed).count() > stable_hold_duration_ * 1000) {
                        drone->log("Stable hose alignment completed - ready for payload drop");
                        return "ALIGNED";
                    }
                }
            } else {
                // Lost stability
                alignment_stable_ = false;
            }
            
            // Periodic logging
            static int log_counter = 0;
            if (++log_counter % 20 == 0) {  // Log every second at 20Hz
                drone->log("Aligning with hose: pos_x=" + std::to_string(hose_data.position_x) + 
                          ", pos_y=" + std::to_string(hose_data.position_y) +
                          ", stable=" + (alignment_stable_ ? "true" : "false"));
            }
        } else {
            // No valid hose detection - hover and wait
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            
            static int no_detection_counter = 0;
            if (++no_detection_counter % 40 == 0) {  // Log every 2 seconds
                drone->log("No valid hose detection during alignment");
            }
        }
        
        return "";  // Continue in this state
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Stop motion
        drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        
        // Store final hose position for payload drop
        if (isHoseDetectionValid()) {
            auto hose_data = drone->getHoseDetection();
            blackboard.set<double>("final_hose_x", hose_data.position_x);
            blackboard.set<double>("final_hose_y", hose_data.position_y);
        }
        
        drone->log("Hose alignment completed - ready for payload drop");
    }
};

#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/Detection.hpp"
#include "fase2/PidController.hpp"
#include <chrono>
#include <cmath>

class AlignHoseState : public fsm::State {
public:
    AlignHoseState() : fsm::State(), 
        pid_x_(nullptr), pid_y_(nullptr), pid_yaw_(nullptr) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: ALIGN HOSE");

        // Get alignment parameters from params.yaml
        alignment_timeout = *blackboard.get<double>("align_timeout");
        lost_timeout = *blackboard.get<double>("lost_timeout");
        distance_tolerance = *blackboard.get<double>("dist_tolerance");
        yaw_tolerance = *blackboard.get<double>("yaw_tolerance");
        payload_drop_delay = *blackboard.get<double>("payload_drop_delay");
        
        // Get PID parameters from params.yaml
        float horizontal_kp = *blackboard.get<double>("horizontal_kp");
        float horizontal_ki = *blackboard.get<double>("horizontal_ki");
        float horizontal_kd = *blackboard.get<double>("horizontal_kd");

        float yaw_kp = *blackboard.get<double>("yaw_kp");
        float yaw_ki = *blackboard.get<double>("yaw_ki");
        float yaw_kd = *blackboard.get<double>("yaw_kd");
        
        // Initialize PID controllers (setpoint at image center: 0.5 for x)
        pid_x_ = std::make_unique<PidController>(horizontal_kp, horizontal_ki, horizontal_kd, 0.5);
        pid_y_ = std::make_unique<PidController>(horizontal_kp, horizontal_ki, horizontal_kd, 0.5);
        pid_yaw_ = std::make_unique<PidController>(yaw_kp, yaw_ki, yaw_kd, 0.0);

        pos = drone->getLocalPosition();
        
        start_time = std::chrono::steady_clock::now();
        stable_start_time = std::chrono::steady_clock::now();
        is_stable = false;
        stable_count = 0;
        lost_detection_count = 0;
        
        drone->log("Starting hose alignment with PID control");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        pos = drone->getLocalPosition();
        float current_yaw = drone->getOrientation()[2];

        // Check for timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (elapsed.count() > alignment_timeout) {
            drone->log("Hose alignment timeout");
            return "TIMEOUT";
        }

        auto hose_detections = drone->getMangueiraDetections();
        Detection detection(hose_detections);

        if (!detection.isThereDetection() || 
            detection.getClosestBbox().confidence < 0.5) {  // Use reasonable confidence threshold
            lost_detection_count++;
            if (lost_detection_count > lost_timeout) {  // Lost detections for too long
                drone->log("Lost hose detections for too long");
                return "LOST DETECTIONS";
            }
            // Hold current position if no detections
            drone->setLocalPosition(pos[0], pos[1], pos[2], current_yaw);
            return "";
        }

        lost_detection_count = 0;
        
        auto bbox = detection.getClosestBbox();
        
        float hose_angle = drone->getMangueiraAngle();
        
        // Compute PID outputs
        float pid_output_x = pid_x_->compute(bbox.center_x);
        float pid_output_y = pid_y_->compute(bbox.center_y);
        float pid_output_yaw = pid_yaw_->compute(hose_angle);
        
        
        // Decompose velocities based on yaw (similar to align_home_state)
        float drift_x = pid_output_y * std::cos(-current_yaw) - pid_output_x * std::sin(-current_yaw);
        
        float new_x = pos[0] + drift_x;
        float new_yaw = current_yaw + pid_output_yaw;
        
        // Check if alignment is stable
        float position_error = std::abs(bbox.center_x - 0.5);  // Only check X position error
        float angle_error = std::abs(hose_angle);
        
        if (position_error < distance_tolerance && angle_error < yaw_tolerance) {
            if (!is_stable) {
                is_stable = true;
                stable_start_time = current_time;
                stable_count = 0;
            }
            stable_count++;
            
            auto stable_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - stable_start_time);
            
            if (stable_elapsed.count() > payload_drop_delay * 1000) {
                drone->log("Hose alignment achieved! Dropping payload.");
                return "ALIGNED";
            }
        } else {
            is_stable = false;
            stable_count = 0;
        }
        
        // Apply the computed position
        drone->setLocalPosition(new_x, pos.y(), pos.z(), new_yaw);
        
        // Log progress occasionally
        if (stable_count % 20 == 0) {
            drone->log("Aligning - Pos error: " + std::to_string(position_error) + 
                      ", Angle error: " + std::to_string(angle_error));
        }
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting hose alignment state");
    }

private:
    Drone* drone;
    double alignment_timeout;
    double lost_timeout;
    double distance_tolerance;
    double yaw_tolerance;
    double payload_drop_delay;
    Eigen::Vector3d pos;
    
    std::unique_ptr<PidController> pid_x_, pid_y_, pid_yaw_;
    
    std::chrono::steady_clock::time_point start_time, stable_start_time;
    bool is_stable;
    int stable_count;
    int lost_detection_count;
};
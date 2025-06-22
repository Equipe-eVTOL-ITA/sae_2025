#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/Detection.hpp"
#include "fase2/PidController.hpp"
#include <chrono>
#include <cmath>

class AlignHomeState : public fsm::State {
public:
    AlignHomeState() : fsm::State(),
        pid_x_(nullptr), pid_y_(nullptr) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: ALIGN HOME");

        dist_tolerance = *blackboard.get<double>("dist_tolerance");
        home_timeout = *blackboard.get<double>("home_timeout");
        lost_timeout = *blackboard.get<double>("lost_timeout");
        
        float horizontal_kp = *blackboard.get<double>("horizontal_kp");
        float horizontal_ki = *blackboard.get<double>("horizontal_ki");
        float horizontal_kd = *blackboard.get<double>("horizontal_kd");
        
        pid_x_ = std::make_unique<PidController>(horizontal_kp, horizontal_ki, horizontal_kd, 0.5);
        pid_y_ = std::make_unique<PidController>(horizontal_kp, horizontal_ki, horizontal_kd, 0.5);
                
        start_time = std::chrono::steady_clock::now();
        lost_detection_count = 0;
        
        drone->log("Starting home alignment using blue base detection");
    }
    
    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos = drone->getLocalPosition();
        float yaw = drone->getOrientation()[2];
        
        // Check for timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (elapsed.count() > home_timeout) {
            drone->log("Home alignment timeout - proceeding to landing");
            return "LAND NOW";
        }
        
        // Process blue base detections
        auto blue_detections = drone->getBlueDetections();
        Detection detection(blue_detections);
        if (!detection.isThereDetection()) {
            lost_detection_count++;
            if (lost_detection_count > lost_timeout) {  // Lost detections
                drone->log("Lost blue base detections - proceeding to emergency landing");
                return "LAND NOW";
            }
            // Hold position if no detections
            drone->setLocalPosition(pos[0], pos[1], pos[2], yaw);
            return "";
        }

        // Reset lost detection counter
        lost_detection_count = 0;

        auto bbox = detection.getClosestBbox();
        
        // Calculate distance from center
        float distance_center = detection.getMinDistance();
        
        // Check if within tolerance
        if (distance_center < dist_tolerance) {
            drone->log("Home alignment achieved! Ready for final landing.");
            return "LAND NOW";
        }
        
        // Compute PID outputs
        float pid_output_x = pid_x_->compute(bbox.center_x);
        float pid_output_y = pid_y_->compute(bbox.center_y);

        
        // Decompose velocities based on yaw
        
        float drift_x = std::cos(-yaw) * pid_output_y - std::sin(-yaw) * pid_output_x;
        float drift_y = std::sin(-yaw) * pid_output_y + std::cos(-yaw) * pid_output_x;
        
        float new_x = pos.x() + drift_x;
        float new_y = pos.y() + drift_y;
        
        // Apply the computed position (maintain current height)
        drone->setLocalPosition(new_x, new_y, pos[2], yaw);
        
        // Log progress occasionally
        static int log_counter = 0;
        if (++log_counter % 20 == 0) {
            drone->log("Home alignment - Distance from center: " + std::to_string(distance_center));
        }
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting home alignment state - proceeding to landing");
    }

private:
    Drone* drone;
    double dist_tolerance;
    double home_timeout;
    double lost_timeout;
    Eigen::Vector3d pos;
    
    std::unique_ptr<PidController> pid_x_, pid_y_;
    
    std::chrono::steady_clock::time_point start_time;
    int lost_detection_count;
};
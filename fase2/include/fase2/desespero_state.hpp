#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>
#include <cmath>

class DesesperoState : public fsm::State {
public:
    DesesperoState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: DESESPERO (Desperation Mode)");

        // Get parameters from params.yaml
        max_horizontal_velocity = *blackboard.get<double>("max_horizontal_velocity");
        drop_height = *blackboard.get<double>("drop_height");
        estimated_distance = *blackboard.get<double>("estimated_distance");
        
        yaw = -M_PI / 2;  // Keep pointing forward
        
        pos = drone->getLocalPosition();
        
        // Set target position to estimated hose location
        target_position = Eigen::Vector3d(estimated_distance, 0.0, drop_height);
        
        // State machine for desperation mode
        phase = NAVIGATE_TO_DROP;
        start_time = std::chrono::steady_clock::now();
        
        drone->log("Navigating to estimated drop position: (" + 
                  std::to_string(target_position[0]) + ", " + 
                  std::to_string(target_position[1]) + ", " + 
                  std::to_string(target_position[2]) + ")");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos = drone->getLocalPosition();
        
        switch (phase) {
            case NAVIGATE_TO_DROP: {
                // Navigate to estimated drop position
                Eigen::Vector3d diff = target_position - pos;
                
                if (diff.norm() < 0.2) {
                    // Arrived at drop position
                    phase = DROP_PAYLOAD;
                    drop_start_time = std::chrono::steady_clock::now();
                    drone->log("Arrived at drop position - dropping payload");
                } else {
                    // Continue moving to target
                    Eigen::Vector3d velocity = diff.norm() > max_horizontal_velocity ? 
                                               diff.normalized() * max_horizontal_velocity : diff;
                    Eigen::Vector3d next_pos = pos + velocity;
                    
                    drone->setLocalPosition(next_pos[0], next_pos[1], next_pos[2], yaw);
                }
                break;
            }
            
            case DROP_PAYLOAD: {
                // Drop the payload (gancho)
                drone->dropGancho();
                
                // Wait a moment to ensure drop is completed
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - drop_start_time);
                
                if (elapsed.count() > 1000) {  // Wait 1 second
                    drone->log("GANCHO DROPPED in desperation mode");
                    return "DROPPED GANCHO";
                }
                
                // Hold position while dropping
                drone->setLocalPosition(pos[0], pos[1], pos[2], yaw);
                break;
            }
        }
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting desperation state");
    }

private:
    enum Phase {
        NAVIGATE_TO_DROP,
        DROP_PAYLOAD
    };
    
    Drone* drone;
    double max_horizontal_velocity;
    double drop_height;
    double estimated_distance;
    float yaw;
    Eigen::Vector3d pos, target_position;
    Phase phase;
    std::chrono::steady_clock::time_point start_time, drop_start_time;
};
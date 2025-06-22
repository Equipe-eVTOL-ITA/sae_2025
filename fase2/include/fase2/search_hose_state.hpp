#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/Detection.hpp"
#include <chrono>
#include <cmath>

class SearchHoseState : public fsm::State {
public:
    SearchHoseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: SEARCH HOSE");

        // Get search parameters from params.yaml
        max_horizontal_velocity = *blackboard.get<double>("max_horizontal_velocity");
        search_timeout = *blackboard.get<double>("align_timeout");  // Use align_timeout for search
        
        // Set search height and distances from params.yaml
        search_height = drone->getLocalPosition()[2];
        yaw = -M_PI / 2;  // Keep pointing forward (rotated -90 degrees)
        
        // Initialize search parameters from params.yaml
        search_distance_min = *blackboard.get<double>("min_distance");
        search_distance_max = *blackboard.get<double>("max_distance");
        search_step = 0.2;  // Small step for precise search
        
        pos = drone->getLocalPosition();
        start_time = std::chrono::steady_clock::now();
        
        // Start searching from minimum distance
        current_search_distance = search_distance_min;
        search_direction = 1;  // 1 for increasing distance, -1 for decreasing
        
        drone->log("Starting hose search from distance: " + std::to_string(current_search_distance));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        // Check for timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (elapsed.count() > search_timeout) {
            drone->log("Hose search timeout - proceeding to desperation mode");
            return "HOSE_NOT_FOUND";
        }
        
        // Check for hose detection
        auto hose_detections = drone->getMangueiraDetections();
        if (!hose_detections.empty()) {
            Detection detection(hose_detections);
            if (detection.isThereDetection() && detection.getClosestBbox().confidence >= 0.2) {  // Use confidence from mangueira_detector params
                drone->log("Hose detected! Confidence: " + std::to_string(detection.getClosestBbox().confidence));
                return "HOSE_DETECTED";
            }
        }
        
        // Execute search pattern - oscillate between min/max distances
        pos = drone->getLocalPosition();
        
        // Calculate target position for current search distance
        Eigen::Vector3d target = Eigen::Vector3d(current_search_distance, 0.0, search_height);
        
        // Check if we've reached current search position
        Eigen::Vector3d diff = target - pos;
        if (diff.norm() < 0.15) {
            // We've reached current position, move to next search distance
            current_search_distance += search_direction * search_step;
            
            // Check bounds and reverse direction if needed
            if (current_search_distance >= search_distance_max) {
                current_search_distance = search_distance_max;
                search_direction = -1;
            } else if (current_search_distance <= search_distance_min) {
                current_search_distance = search_distance_min;
                search_direction = 1;
            }
        }
        
        // Move towards target position
        Eigen::Vector3d velocity = diff.norm() > max_horizontal_velocity ? 
                                   diff.normalized() * max_horizontal_velocity : diff;
        Eigen::Vector3d next_pos = pos + velocity;
        
        drone->setLocalPosition(next_pos[0], next_pos[1], next_pos[2], yaw);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting hose search state");
    }

private:
    Drone* drone;
    double max_horizontal_velocity;
    double search_timeout;
    double search_height;
    double search_distance_min, search_distance_max, search_step;
    double current_search_distance;
    int search_direction;
    float yaw;
    Eigen::Vector3d pos;
    std::chrono::steady_clock::time_point start_time;
};
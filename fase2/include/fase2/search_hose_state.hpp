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
        max_velocity = *blackboard.get<double>("max_horizontal_velocity");
        search_timeout = *blackboard.get<double>("align_timeout");  // Use align_timeout for search
        
        // Set search height and distances from params.yaml
        search_height = drone->getLocalPosition()[2];
        yaw = drone->getOrientation()[2];
        
        // Initialize search parameters from params.yaml
        search_distance_min = *blackboard.get<double>("min_distance");
        search_distance_max = *blackboard.get<double>("max_distance");
        
        pos = drone->getLocalPosition();
        start_time = std::chrono::steady_clock::now();
        
        current_search_distance = search_distance_max;

        first_pass = false;
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
        
        pos = drone->getLocalPosition();        
        Eigen::Vector3d target = Eigen::Vector3d(current_search_distance, 0.0, search_height);
        Eigen::Vector3d diff = target - pos;

        if (diff.norm() < 0.10){
            if (first_pass){
                drone->log("Done full forward and backwards movement, hose not found.");
                return "HOSE_NOT_FOUND";
            }
            else{
                first_pass = true;
                current_search_distance = search_distance_min;
                drone->log("Switching to backward search");
            }
        }

        target = Eigen::Vector3d(current_search_distance, 0.0, search_height);
        diff = target - pos;
        
        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);
        
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], yaw);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    Drone* drone;
    bool first_pass;
    double max_velocity;
    double search_timeout;
    double search_height;
    double search_distance_min, search_distance_max;
    double current_search_distance;
    float yaw;
    Eigen::Vector3d pos;
    std::chrono::steady_clock::time_point start_time;
};
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
        drone->log("STATE: DESESPERO");

        // Get parameters from params.yaml
        max_velocity = *blackboard.get<double>("max_horizontal_velocity");
        double drop_height = *blackboard.get<double>("drop_height");
        estimated_distance = *blackboard.get<double>("estimated_distance");
        dist_tolerance = *blackboard.get<double>("dist_tolerance");
        
        yaw = -M_PI / 2;  // Keep pointing forward
                
        // Set target position to estimated hose location
        goal = Eigen::Vector3d(estimated_distance, 0.0, drop_height);
        
        start_time = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        Eigen::Vector3d diff = goal - pos;

        if (diff.norm() < dist_tolerance) {
            return "AT ESTIMATE";
        }
        
        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], yaw);

        return "";
    }

private:
    Drone* drone;
    double max_velocity;
    double estimated_distance;
    float yaw;
    double dist_tolerance;
    Eigen::Vector3d pos, goal;
    std::chrono::steady_clock::time_point start_time, drop_start_time;
};
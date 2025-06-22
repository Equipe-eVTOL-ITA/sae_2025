#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>

class GoToState : public fsm::State {
public:
    GoToState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GO TO");

        max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        float estimated_distance = *blackboard.get<float>("estimated_distance");

        goal = Eigen::Vector3d(estimated_distance, 0.0, takeoff_height);

        yaw = -M_PI / 2;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        Eigen::Vector3d diff = goal - pos;

        if (diff.norm() < 0.08) {
            return "ARRIVED";
        }
        
        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    Eigen::Vector3d pos;
    Drone* drone;
    Eigen::Vector3d goal;
    float max_velocity;
    float yaw;
};
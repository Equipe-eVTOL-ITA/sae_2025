#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>

class RotateState : public fsm::State {
public:
    RotateState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: ROTATE");

        double takeoff_height = *blackboard.get<double>("takeoff_height");

        goal = Eigen::Vector3d(0.0, 0.0, takeoff_height);

        yaw_tolerance = *blackboard.get<double>("yaw_tolerance");

        goal_yaw = -M_PI / 2;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        float yaw = drone->getOrientation()[2];

        float diff = goal_yaw - yaw;

        if (std::abs(diff) < yaw_tolerance) {
            return "ROTATED";
        }

        float step = (std::abs(diff) > 0.13 ? (diff > 0 ? 0.13 : -0.13) : diff);
        float little_yaw = yaw + step;
        
        drone->setLocalPosition(goal[0], goal[1], goal[2], little_yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    Drone* drone;
    Eigen::Vector3d goal;
    float goal_yaw;
    double yaw_tolerance;
};
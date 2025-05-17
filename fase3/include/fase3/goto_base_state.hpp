#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GoToBaseState");

        max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        float takeoff_height = *blackboard.get<float>("takeoff_height");
        
        const Eigen::Vector2d approx_base = *blackboard.get<Eigen::Vector2d>("approximate_base");
        goal = Eigen::Vector3d(approx_base.x(), approx_base.y(), takeoff_height);

        yaw = drone->getOrientation()[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.08) {
            return "ARRIVED AT BASE";
        }
        
        Eigen::Vector3d diff = goal - pos;
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
#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: RETURN HOME");

        max_velocity = *blackboard.get<double>("max_horizontal_velocity");
        double return_height = *blackboard.get<double>("return_height");
        dist_tolerance = *blackboard.get<double>("dist_tolerance");
        
        goal = Eigen::Vector3d(0.0, 0.0, return_height);
        yaw = drone->getOrientation()[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        Eigen::Vector3d diff = goal - pos;

        if (diff.norm() < dist_tolerance) {
            return "ARRIVED_HOME";
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
    double max_velocity;
    double yaw;
    double dist_tolerance;
};
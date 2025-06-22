#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: INITIAL TAKEOFF");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({0.0, 0.0, 0.0});
        print_counter = 0;
        
        double takeoff_height = *blackboard.get<double>("takeoff_height");
        max_velocity = *blackboard.get<double>("max_vertical_velocity");
        dist_tolerance = *blackboard.get<double>("dist_tolerance");
        
        drone->toOffboardSync();
        drone->armSync();
        drone->setHomePosition(fictual_home);
                
        pos = drone->getLocalPosition();
        initial_yaw = drone->getOrientation()[2];
        goal = Eigen::Vector3d({pos[0], pos[1], takeoff_height});

        drone->log("Initial Yaw: " + std::to_string(initial_yaw));
        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if (print_counter%10==0){
            drone->log("Pos: {" + std::to_string(pos[0]) + ", " 
                        + std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + "}");
        }
        print_counter++;

        Eigen::Vector3d diff = goal - pos;
        if (diff.norm() < dist_tolerance){
            return "TAKEOFF_COMPLETED";
        }

        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);
        
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        
        return "";
    }

private:
    double max_velocity;
    Eigen::Vector3d pos, goal, goal_diff, little_goal;
    Drone* drone;
    int print_counter;
    double initial_yaw;
    double dist_tolerance;
};
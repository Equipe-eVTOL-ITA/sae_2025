#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "PidController.hpp"
#include "Detection.hpp"

class PrecisionAlignState : public fsm::State {
public:
    PrecisionAlignState() : fsm::State(), drone(nullptr),
                            x_pid(0.9, 0.0, 0.05, 0.5),
                            y_pid(0.9, 0.0, 0.05, 0.5) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (!drone)
            return;

        drone->log("STATE: PrecisionAlignState");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        float x_rate = 0.0, y_rate = 0.0;
        float vertical_distance = 1.0;

        float yaw = drone->getOrientation()[2];

        Detection verticalDetection(drone->getVerticalBboxes());

        if (verticalDetection.isThereDetection()){
            DronePX4::BoundingBox vertical_bbox = verticalDetection.getClosestBbox();
            vertical_distance = verticalDetection.getMinDistance();

            x_rate = x_pid.compute(-vertical_bbox.center_y);
            y_rate = y_pid.compute(vertical_bbox.center_x);
        }
        if (vertical_distance < 0.03){
            return "ALIGNED";
        }

        float frd_x_rate = x_rate * cos(yaw) - y_rate * sin(yaw);
        float frd_y_rate = x_rate * sin(yaw) + y_rate * cos(yaw);
        drone->setLocalVelocity(frd_x_rate, frd_y_rate, 0, 0);

        return "";
    }

private:
    Drone* drone;
    int waypoints_visited;

    PidController x_pid, y_pid;
};
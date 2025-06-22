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
    PrecisionAlignState() : fsm::State(), drone(nullptr), x_pid(0,0,0,0), y_pid(0,0,0,0) {}

    void on_enter(fsm::Blackboard &blackboard){
        this->drone = blackboard.get<Drone>("drone");
        if(!this->drone) return;
        this->drone->log("STATE: Alinhando com a base no chao...");

        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->kp = *blackboard.get<float>("pid_pos_kp");
        this->ki = *blackboard.get<float>("pid_pos_ki");
        this->kd = *blackboard.get<float>("pid_pos_kd");
        this->setpoint = *blackboard.get<float>("setpoint");

        this->x_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
        this->y_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
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
        if (vertical_distance < this->position_tolerance){
            return "OVER THE BASE";
        }

        float frd_x_rate = x_rate * cos(yaw) - y_rate * sin(yaw);
        float frd_y_rate = x_rate * sin(yaw) + y_rate * cos(yaw);
        drone->setLocalVelocity(frd_x_rate, frd_y_rate, 0, 0);

        return "";
    }

private:
    Drone* drone;
    PidController x_pid, y_pid;
    float kp, ki, kd;
    float setpoint;
    float position_tolerance;
};
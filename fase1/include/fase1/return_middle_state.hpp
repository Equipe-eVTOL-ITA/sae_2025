#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class ReturnMiddleState : public fsm::State {
public:
    ReturnMiddleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        // Check if we completed all poles or came here due to POLE_NOT_FOUND
        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        if (current_pole >= total_poles) {
            drone->log("STATE: Returning to middle position - ALL POLES COMPLETED");
            mission_complete = true;
        } else {
            drone->log("STATE: Returning to middle position - POLE NOT FOUND");
            mission_complete = false;
        }

        // Get return parameters
        return_speed = *blackboard.get<float>("return_speed");
        position_tolerance = *blackboard.get<float>("position_tolerance");
        max_return_time = *blackboard.get<float>("max_return_time");
        
        // Get current position
        goal = drone->getLocalPosition();
        yaw =  drone->getOrientation()[2];

        goal.x() += 0.4;
        goal.y() = 0;
        
        start_time = std::chrono::high_resolution_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        
        // Check for timeout
        if (elapsed.count() > max_return_time) {
            drone->log("Return timeout - proceeding to next state");
            if (mission_complete) {
                return "RETURN_TIMEOUT_COMPLETE";
            } else {
                return "RETURN_TIMEOUT_LOST";
            }
        }

        // Get current position
        Eigen::Vector3d pos = drone->getLocalPosition();
        
        Eigen::Vector3d diff = goal - pos;
        Eigen::Vector3d little_goal = pos + (diff.norm() > return_speed ? diff.normalized() * return_speed : diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], yaw);

        if ((diff.norm() < position_tolerance)) {
            drone->log("Reached middle position");
            if (mission_complete) {
                drone->log("All poles completed - proceeding to landing");
                return "RETURN_COMPLETE";
            } else {
                drone->log("Not all poles completed - entering lost state");
                return "RETURN_TO_LOST";
            }
        }

        return "";
    }


private:
    Drone* drone{nullptr};
    int current_pole;
    int total_poles;
    bool mission_complete;
    float return_speed;
    float position_tolerance;
    float max_return_time;
    float yaw;
    Eigen::Vector3d goal;    
    std::chrono::high_resolution_clock::time_point start_time;
};

#pragma once

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class CheckNextPoleState : public fsm::State {
public:
    CheckNextPoleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        current_pole = *blackboard.get<int>("current_pole");
        total_poles = *blackboard.get<int>("total_poles");
        
        drone->log("STATE: Checking mission progress - completed pole " + 
                  std::to_string(current_pole) + "/" + std::to_string(total_poles));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (current_pole < total_poles) {
            drone->log("Continuing to next pole: " + std::to_string(current_pole + 1));
            return "CONTINUE_MISSION";
        } else {
            drone->log("All poles completed - mission complete!");
            return "MISSION_COMPLETE";
        }
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    Drone* drone{nullptr};
    int current_pole;
    int total_poles;
};

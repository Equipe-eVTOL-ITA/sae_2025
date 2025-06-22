#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "ArenaPoint.hpp"

class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State() {}

    // A ideia do Blackboard eh criar um canal de troca de informacoes entre os estados, que estao isolados pela FSM
    void on_enter(fsm::Blackboard &blackboard) override {
        
        this->drone = blackboard.get<Drone>("drone"); // obtendo o ponteiro para o objeto drone
        if(this->drone == nullptr) return;
        drone->log("STATE: Procurando bases...");

        this->initial_yaw = *blackboard.get<float>("initial_yaw");
        this->waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        this->class_id = *blackboard.get<std::string>("class_id");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        this->pos = this->drone->getLocalPosition();
        this->goal_point = this->getNextPoint(this->waypoints);

        if(this->goal_point == nullptr) return "BASE FOUND";
        
        this->goal = this->goal_point->coordinates;
        this->goal_diff = this->goal - this->pos;

        if (goal_diff.norm() < 0.08) {
            this->goal_point->is_visited = true; // marca o ponto como visitado
            this->drone->log("Ponto visitado.");
            return ""; // continua procurando
        }



        if(this->goal_diff.norm() > this->max_velocity)
            this->goal_diff = this->goal_diff.normalized() * this->max_velocity;

        Eigen::Vector3d little_goal = this->goal_diff + this->pos;

        this->drone->setLocalPosition(
            little_goal[0],
            little_goal[1],
            little_goal[2],
            this->initial_yaw
        );

        auto bboxes = drone->getVerticalBboxes();
        if (!bboxes.empty()) {
            int temp_counter = 0;
            for (const auto& bbox : bboxes) {
                if (bbox.class_id == this->class_id) {
                    this->drone->log("Base " + this->class_id + " encontrada.");
                    this->counter++;
                    temp_counter++;
                }
            }
            if(temp_counter==0)
                this->counter = 0;
            if(this->counter >= 5){ // espera detectar 5 vezes antes de passar para a proxima etapa
                this->counter = 0;
                return "BASE FOUND";
            }
        }

        return "";

    }

    void on_exit(fsm::Blackboard &blackboard) override {
        this->pos = this->drone->getLocalPosition();
        blackboard.set<Eigen::Vector3d>("last search position", pos);
    }

private:
    Drone* drone;
    std::vector<Base>* bases;
    std::vector<DronePX4::BoundingBox> bboxes, previous_bboxes;
    std::vector<ArenaPoint>* waypoints;
    ArenaPoint* goal_point;
    Eigen::Vector3d pos, goal, goal_diff;
    const float max_velocity = 0.4;
    float initial_yaw;
    std::string class_id;
    
    int counter = 0;

    ArenaPoint* getNextPoint(std::vector<ArenaPoint>* waypoints) {
        if (!waypoints) return nullptr;
        for (auto& point : *waypoints) {
            if (!point.is_visited) {
                return &point;
            }
        }
        // Return nullptr if no unvisited points are found
        return nullptr;
    }

};
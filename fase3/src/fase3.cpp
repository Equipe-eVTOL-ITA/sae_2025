#include "fase3/initial_takeoff_state.hpp"
#include "fase3/landing_state.hpp"
#include "fase3/precision_align_state.hpp"
#include "fase3/goto_base_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class Fase3FSM : public fsm::FSM {
public:
    Fase3FSM(float takeoff_height, float max_vertical_velocity, 
             float max_horizontal_velocity, int classe_base_pouso) 
        : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        
        // PARAMETERS

        this->blackboard_set<float>("takeoff_height", takeoff_height);
        this->blackboard_set<float>("max_vertical_velocity", max_vertical_velocity);
        this->blackboard_set<float>("max_horizontal_velocity", max_horizontal_velocity);
        this->blackboard_set<int>("classe_base_pouso", classe_base_pouso);

        // STATES

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("GO TO POSITION", std::make_unique<GoToBaseState>());
        this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "SEARCH BASE"},{"SEG FAULT", "ERROR"}});

        // Search Base transitions
        this->add_transitions("SEARCH BASE", {{"FOUND BASE", "GO TO BASE"},{"SEG FAULT", "ERROR"}});

        // Go To Base transitions
        this->add_transitions("GO TO BASE", {{"ARRIVED AT BASE", "PRECISION ALIGN"},{"SEG FAULT", "ERROR"}});

        // Precision Align transitions
        this->add_transitions("PRECISION ALIGN", {{"ALIGNED", "PRECISION LANDING"},{"SEG FAULT", "ERROR"}});

        // Precision Landing transitions
        this->add_transitions("PRECISION LANDING", {{"LANDED", "FINISHED"},{"SEG FAULT", "ERROR"}});
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase3_node") {
        // Declare parameters in the Node class
        this->declare_parameter("takeoff_height", -2.0);
        this->declare_parameter("max_vertical_velocity", 0.8);
        this->declare_parameter("max_horizontal_velocity", 0.8);
        this->declare_parameter("classe_base_pouso", 2);

        // Get parameter values
        float takeoff_height = this->get_parameter("takeoff_height").as_double();
        float max_vertical_velocity = this->get_parameter("max_vertical_velocity").as_double();
        float max_horizontal_velocity = this->get_parameter("max_horizontal_velocity").as_double();
        int classe_base_pouso = this->get_parameter("classe_base_pouso").as_int();
        
        // Initialize the FSM with the parameters
        my_fsm = std::make_unique<Fase3FSM>(
            takeoff_height, 
            max_vertical_velocity, 
            max_horizontal_velocity, 
            classe_base_pouso
        );
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !my_fsm->is_finished()) {
            my_fsm->execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    std::unique_ptr<Fase3FSM> my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
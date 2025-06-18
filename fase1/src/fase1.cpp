#include "fase1/initial_takeoff_state.hpp"
#include "fase1/search_pole_state.hpp"
#include "fase1/approach_pole_state.hpp"
#include "fase1/around_pole_state.hpp"
#include "fase1/return_middle_state.hpp"
#include "fase1/check_next_pole_state.hpp"
#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include "fase1/landing_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>
#include <vector>


class SlalomFSM : public fsm::FSM {
public:
    SlalomFSM(float takeoff_height, float max_vertical_velocity, float max_horizontal_velocity, 
              int total_poles, float rotation_speed, float approach_speed, 
              float navigation_radius, float angular_velocity, float return_speed,
              float max_search_time, float goal_width, float max_approach_time, 
              float centering_tolerance, float pid_yaw_kp, float pid_yaw_ki, 
              float pid_yaw_kd, float pid_distance_kp, float pid_distance_ki, 
              float pid_distance_kd, float max_navigation_time, 
              float position_tolerance, float max_return_time,
              const std::vector<std::string>& pole_colors) 
        : fsm::FSM({"ERROR", "FINISHED"}) {

        Drone* drone = new Drone();
        this->blackboard_set<Drone>("drone", drone);

        drone->printSubscriptionStats();
        
        // CORE PARAMETERS
        this->blackboard_set<float>("takeoff_height", takeoff_height);
        this->blackboard_set<float>("max_vertical_velocity", max_vertical_velocity);
        this->blackboard_set<float>("max_horizontal_velocity", max_horizontal_velocity);
        
        // SLALOM PARAMETERS
        this->blackboard_set<int>("total_poles", total_poles);
        this->blackboard_set<int>("current_pole", 0); // Start with pole 0
        this->blackboard_set<float>("rotation_speed", rotation_speed);
        this->blackboard_set<float>("approach_speed", approach_speed);
        this->blackboard_set<float>("navigation_radius", navigation_radius);
        this->blackboard_set<float>("angular_velocity", angular_velocity);
        this->blackboard_set<float>("return_speed", return_speed);
        
        // SEARCH PARAMETERS
        this->blackboard_set<float>("max_search_time", max_search_time);
        
        // APPROACH PARAMETERS  
        this->blackboard_set<float>("goal_width", goal_width);
        this->blackboard_set<float>("max_approach_time", max_approach_time);
        this->blackboard_set<float>("centering_tolerance", centering_tolerance);
        
        // PID PARAMETERS FOR APPROACH
        this->blackboard_set<float>("pid_yaw_kp", pid_yaw_kp);
        this->blackboard_set<float>("pid_yaw_ki", pid_yaw_ki);
        this->blackboard_set<float>("pid_yaw_kd", pid_yaw_kd);
        this->blackboard_set<float>("pid_distance_kp", pid_distance_kp);
        this->blackboard_set<float>("pid_distance_ki", pid_distance_ki);
        this->blackboard_set<float>("pid_distance_kd", pid_distance_kd);
        
        // NAVIGATION PARAMETERS
        this->blackboard_set<float>("max_navigation_time", max_navigation_time);
        
        // RETURN PARAMETERS
        this->blackboard_set<float>("position_tolerance", position_tolerance);
        this->blackboard_set<float>("max_return_time", max_return_time);
        
        // POLE DIRECTIONS (alternating sides: right, left, right, left)
        std::vector<bool> pole_directions = {true, false, true, false}; // true = right, false = left
        this->blackboard_set<std::vector<bool>>("pole_directions", pole_directions);
        
        // POLE COLORS (sequence of colors to traverse)
        this->blackboard_set<std::vector<std::string>>("pole_colors", pole_colors);

        // STATES
        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH POLE", std::make_unique<SearchPoleState>());
        this->add_state("APPROACH POLE", std::make_unique<ApproachPoleState>());
        this->add_state("AROUND POLE", std::make_unique<AroundPoleState>());
        this->add_state("RETURN MIDDLE", std::make_unique<ReturnMiddleState>());
        this->add_state("FINAL LANDING", std::make_unique<LandingState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {
            {"INITIAL TAKEOFF COMPLETED", "SEARCH POLE"},
            {"SEG FAULT", "ERROR"}
        });

        // Search Pole transitions  
        this->add_transitions("SEARCH POLE", {
            {"POLE_FOUND", "APPROACH POLE"},
            {"POLE_NOT_FOUND", "RETURN MIDDLE"}, // Skip to return if pole not found
            {"SEG FAULT", "ERROR"}
        });

        // Approach Pole transitions
        this->add_transitions("APPROACH POLE", {
            {"APPROACH_COMPLETE", "AROUND POLE"},
            {"APPROACH_TIMEOUT", "AROUND POLE"}, // Proceed even if not perfectly aligned
            {"POLE_LOST", "SEARCH POLE"}, // Return to search if pole lost
            {"SEG FAULT", "ERROR"}
        });

        // Around Pole transitions
        this->add_transitions("AROUND POLE", {
            {"NAVIGATION_COMPLETE", "CHECK_NEXT_POLE"},
            {"NAVIGATION_TIMEOUT", "CHECK_NEXT_POLE"}, // Proceed to next even on timeout
            {"SEG FAULT", "ERROR"}
        });

        // Virtual state to check if we should continue to next pole or finish
        this->add_state("CHECK_NEXT_POLE", std::make_unique<CheckNextPoleState>());
        this->add_transitions("CHECK_NEXT_POLE", {
            {"CONTINUE_MISSION", "SEARCH POLE"},
            {"MISSION_COMPLETE", "RETURN MIDDLE"},
            {"SEG FAULT", "ERROR"}
        });

        // Return Middle transitions
        this->add_transitions("RETURN MIDDLE", {
            {"RETURN_COMPLETE", "FINAL LANDING"},
            {"RETURN_TIMEOUT", "FINAL LANDING"}, // Land even if not perfectly centered
            {"SEG FAULT", "ERROR"}
        });

        // Final Landing transitions
        this->add_transitions("FINAL LANDING", {
            {"LANDED", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase1_fsm") {
        // Declare basic parameters
        this->declare_parameter("takeoff_height", -2.0);
        this->declare_parameter("max_vertical_velocity", 0.8);
        this->declare_parameter("max_horizontal_velocity", 0.8);
        
        // Declare slalom mission parameters
        this->declare_parameter("total_poles", 4);
        this->declare_parameter("rotation_speed", 0.5);
        this->declare_parameter("approach_speed", 0.5);
        this->declare_parameter("navigation_radius", 1.5);
        this->declare_parameter("angular_velocity", 0.8);
        this->declare_parameter("return_speed", 1.0);
        
        // Declare search parameters
        this->declare_parameter("max_search_time", 30.0);
        
        // Declare approach parameters
        this->declare_parameter("goal_width", 0.3);
        this->declare_parameter("max_approach_time", 20.0);
        this->declare_parameter("centering_tolerance", 0.05);
        
        // Declare PID parameters
        this->declare_parameter("pid_yaw_kp", 1.0);
        this->declare_parameter("pid_yaw_ki", 0.1);
        this->declare_parameter("pid_yaw_kd", 0.05);
        this->declare_parameter("pid_distance_kp", 0.5);
        this->declare_parameter("pid_distance_ki", 0.1);
        this->declare_parameter("pid_distance_kd", 0.02);
        
        // Declare navigation parameters
        this->declare_parameter("max_navigation_time", 45.0);
        
        // Declare return parameters
        this->declare_parameter("position_tolerance", 0.3);
        this->declare_parameter("max_return_time", 30.0);
        
        // Declare pole color parameters
        this->declare_parameter("color_1", "Rosa");
        this->declare_parameter("color_2", "Vermelho");
        this->declare_parameter("color_3", "Preto");
        this->declare_parameter("color_4", "Azul");

        // Get parameter values
        float takeoff_height = this->get_parameter("takeoff_height").as_double();
        float max_vertical_velocity = this->get_parameter("max_vertical_velocity").as_double();
        float max_horizontal_velocity = this->get_parameter("max_horizontal_velocity").as_double();
        int total_poles = this->get_parameter("total_poles").as_int();
        float rotation_speed = this->get_parameter("rotation_speed").as_double();
        float approach_speed = this->get_parameter("approach_speed").as_double();
        float navigation_radius = this->get_parameter("navigation_radius").as_double();
        float angular_velocity = this->get_parameter("angular_velocity").as_double();
        float return_speed = this->get_parameter("return_speed").as_double();
        
        // Get additional parameters
        float max_search_time = this->get_parameter("max_search_time").as_double();
        float goal_width = this->get_parameter("goal_width").as_double();
        float max_approach_time = this->get_parameter("max_approach_time").as_double();
        float centering_tolerance = this->get_parameter("centering_tolerance").as_double();
        float pid_yaw_kp = this->get_parameter("pid_yaw_kp").as_double();
        float pid_yaw_ki = this->get_parameter("pid_yaw_ki").as_double();
        float pid_yaw_kd = this->get_parameter("pid_yaw_kd").as_double();
        float pid_distance_kp = this->get_parameter("pid_distance_kp").as_double();
        float pid_distance_ki = this->get_parameter("pid_distance_ki").as_double();
        float pid_distance_kd = this->get_parameter("pid_distance_kd").as_double();
        float max_navigation_time = this->get_parameter("max_navigation_time").as_double();
        float position_tolerance = this->get_parameter("position_tolerance").as_double();
        float max_return_time = this->get_parameter("max_return_time").as_double();
        
        // Get pole color parameters
        std::vector<std::string> pole_colors = {
            this->get_parameter("color_1").as_string(),
            this->get_parameter("color_2").as_string(),
            this->get_parameter("color_3").as_string(),
            this->get_parameter("color_4").as_string()
        };
        
        // Initialize the SlalomFSM with all parameters
        my_fsm = std::make_unique<SlalomFSM>(
            takeoff_height, max_vertical_velocity, max_horizontal_velocity, 
            total_poles, rotation_speed, approach_speed, navigation_radius,
            angular_velocity, return_speed, max_search_time, goal_width, 
            max_approach_time, centering_tolerance, pid_yaw_kp, pid_yaw_ki, 
            pid_yaw_kd, pid_distance_kp, pid_distance_ki, pid_distance_kd, 
            max_navigation_time, position_tolerance, max_return_time, pole_colors
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
    std::unique_ptr<SlalomFSM> my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
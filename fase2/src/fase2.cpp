#include "fase2/initial_takeoff_state.hpp"
#include "fase2/rotate_state.hpp"
#include "fase2/goto_state.hpp"
#include "fase2/search_hose_state.hpp"
#include "fase2/desespero_state.hpp"
#include "fase2/align_hose_state.hpp"
#include "fase2/return_home_state.hpp"
#include "fase2/align_home_state.hpp"
#include "fase2/landing_state.hpp"
#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class Fase2FSM : public fsm::FSM {
public:
    Fase2FSM(double takeoff_height, double max_vertical_velocity, double max_horizontal_velocity,
             double estimated_distance, double min_distance, double max_distance,
             double drop_height, double horizontal_kp, double horizontal_ki, double horizontal_kd,
             double yaw_kp, double yaw_ki, double yaw_kd,
             double align_timeout, double dist_tolerance, double yaw_tolerance,
             double return_height, double home_timeout, double lost_timeout, double payload_drop_delay) 
        : fsm::FSM({"ERROR", "FINISHED"}) {

        Drone* drone = new Drone();
        this->blackboard_set<Drone>("drone", drone);

        drone->printSubscriptionStats();
        
        // MISSION PARAMETERS FROM params.yaml
        this->blackboard_set<double>("takeoff_height", takeoff_height);
        this->blackboard_set<double>("max_vertical_velocity", max_vertical_velocity);
        this->blackboard_set<double>("max_horizontal_velocity", max_horizontal_velocity);
        this->blackboard_set<double>("estimated_distance", estimated_distance);
        this->blackboard_set<double>("min_distance", min_distance);
        this->blackboard_set<double>("max_distance", max_distance);
        this->blackboard_set<double>("drop_height", drop_height);
        
        // PID CONTROL PARAMETERS
        this->blackboard_set<double>("horizontal_kp", horizontal_kp);
        this->blackboard_set<double>("horizontal_ki", horizontal_ki);
        this->blackboard_set<double>("horizontal_kd", horizontal_kd);
        this->blackboard_set<double>("yaw_kp", yaw_kp);
        this->blackboard_set<double>("yaw_ki", yaw_ki);
        this->blackboard_set<double>("yaw_kd", yaw_kd);
        
        // TIMEOUT AND TOLERANCE PARAMETERS
        this->blackboard_set<double>("align_timeout", align_timeout);
        this->blackboard_set<double>("dist_tolerance", dist_tolerance);
        this->blackboard_set<double>("yaw_tolerance", yaw_tolerance);
        this->blackboard_set<double>("return_height", return_height);
        this->blackboard_set<double>("home_timeout", home_timeout);
        this->blackboard_set<double>("lost_timeout", lost_timeout);
        this->blackboard_set<double>("payload_drop_delay", payload_drop_delay);

        // STATES
        this->add_state("INITIAL_TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("ROTATE", std::make_unique<RotateState>());
        this->add_state("GO_TO", std::make_unique<GoToState>());
        this->add_state("SEARCH_HOSE", std::make_unique<SearchHoseState>());
        this->add_state("DESESPERO", std::make_unique<DesesperoState>());
        this->add_state("ALIGN_HOSE", std::make_unique<AlignHoseState>());
        this->add_state("RETURN_HOME", std::make_unique<ReturnHomeState>());
        this->add_state("ALIGN_HOME", std::make_unique<AlignHomeState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // TRANSITIONS
        
        this->add_transitions("INITIAL_TAKEOFF", {
            {"TAKEOFF_COMPLETED", "ROTATE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ROTATE", {
            {"ROTATED", "GO_TO"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GO_TO", {
            {"ARRIVED", "SEARCH_HOSE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH_HOSE", {
            {"HOSE_DETECTED", "ALIGN_HOSE"},
            {"HOSE_NOT_FOUND", "DESESPERO"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("DESESPERO", {
            {"DROPPED GANCHO", "RETURN_HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN_HOSE", {
            {"ALIGNED", "RETURN_HOME"},
            {"TIMEOUT", "RETURN_HOME"},
            {"LOST DETECTIONS", "DESESPERO"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("RETURN_HOME", {
            {"ARRIVED_HOME", "ALIGN_HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN_HOME", {
            {"LAND NOW", "LANDING"},
            {"SEG FAULT", "ERROR"}
        });

        // Final Landing transitions
        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase2_node") {
        // Declare mission parameters matching params.yaml
        this->declare_parameter("takeoff_height", -2.8);
        this->declare_parameter("max_vertical_velocity", 1.0);
        this->declare_parameter("max_horizontal_velocity", 1.0);
        this->declare_parameter("estimated_distance", 3.0);
        this->declare_parameter("min_distance", 2.5);
        this->declare_parameter("max_distance", 3.5);
        this->declare_parameter("drop_height", -2.5);
        
        // PID control parameters
        this->declare_parameter("horizontal_kp", 1.0);
        this->declare_parameter("horizontal_ki", 0.05);
        this->declare_parameter("horizontal_kd", 0.2);
        this->declare_parameter("yaw_kp", 1.0);
        this->declare_parameter("yaw_ki", 0.05);
        this->declare_parameter("yaw_kd", 0.2);
        
        // Timeout and tolerance parameters
        this->declare_parameter("align_timeout", 50.0);
        this->declare_parameter("dist_tolerance", 0.07);
        this->declare_parameter("yaw_tolerance", 0.1);
        this->declare_parameter("return_height", -1.5);
        this->declare_parameter("home_timeout", 20.0);
        this->declare_parameter("lost_timeout", 15.0);
        this->declare_parameter("payload_drop_delay", 2.0);

        // Get all parameter values
        double takeoff_height = this->get_parameter("takeoff_height").as_double();
        double max_vertical_velocity = this->get_parameter("max_vertical_velocity").as_double();
        double max_horizontal_velocity = this->get_parameter("max_horizontal_velocity").as_double();
        double estimated_distance = this->get_parameter("estimated_distance").as_double();
        double min_distance = this->get_parameter("min_distance").as_double();
        double max_distance = this->get_parameter("max_distance").as_double();
        double drop_height = this->get_parameter("drop_height").as_double();
        
        double horizontal_kp = this->get_parameter("horizontal_kp").as_double();
        double horizontal_ki = this->get_parameter("horizontal_ki").as_double();
        double horizontal_kd = this->get_parameter("horizontal_kd").as_double();
        double yaw_kp = this->get_parameter("yaw_kp").as_double();
        double yaw_ki = this->get_parameter("yaw_ki").as_double();
        double yaw_kd = this->get_parameter("yaw_kd").as_double();
        
        double align_timeout = this->get_parameter("align_timeout").as_double();
        double dist_tolerance = this->get_parameter("dist_tolerance").as_double();
        double yaw_tolerance = this->get_parameter("yaw_tolerance").as_double();
        double return_height = this->get_parameter("return_height").as_double();
        double home_timeout = this->get_parameter("home_timeout").as_double();
        double lost_timeout = this->get_parameter("lost_timeout").as_double();
        double payload_drop_delay = this->get_parameter("payload_drop_delay").as_double();
        
        // Initialize the FSM with all parameters
        my_fsm = std::make_unique<Fase2FSM>(
            takeoff_height, max_vertical_velocity, max_horizontal_velocity,
            estimated_distance, min_distance, max_distance,
            drop_height, horizontal_kp, horizontal_ki, horizontal_kd,
            yaw_kp, yaw_ki, yaw_kd,
            align_timeout, dist_tolerance, yaw_tolerance,
            return_height, home_timeout, lost_timeout, payload_drop_delay
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
    std::unique_ptr<Fase2FSM> my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
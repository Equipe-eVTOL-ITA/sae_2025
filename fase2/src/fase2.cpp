#include "fase2/initial_takeoff_state.hpp"
#include "fase2/go_to_hose_state.hpp"
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
    Fase2FSM(double takeoff_height, double line_following_height, 
             double forward_velocity, double max_lateral_velocity, double max_yaw_rate,
             double line_confidence_threshold, double hose_confidence_threshold,
             double max_line_following_time, double hose_detection_timeout,
             
             // Line following PID parameters (6 parameters)
             float line_follow_lateral_kp, float line_follow_lateral_ki, float line_follow_lateral_kd,
             float line_follow_height_kp, float line_follow_height_ki, float line_follow_height_kd,
             float line_follow_yaw_kp, float line_follow_yaw_ki, float line_follow_yaw_kd,
             
             double hose_alignment_tolerance, double hose_alignment_velocity,
             double hose_stability_threshold, double hose_alignment_timeout,
             double hose_stable_hold_time, double payload_drop_height,
             
             // Hose alignment PID parameters (6 parameters)
             float hose_align_lateral_kp, float hose_align_lateral_ki, float hose_align_lateral_kd,
             float hose_align_vertical_kp, float hose_align_vertical_ki, float hose_align_vertical_kd,
             float hose_align_yaw_kp, float hose_align_yaw_ki, float hose_align_yaw_kd,
             
             double max_horizontal_velocity, double home_position_tolerance,
             double return_height, double return_navigation_timeout,
             double home_alignment_tolerance, double home_alignment_velocity,
             double home_stability_threshold, double home_alignment_timeout,
             double home_stable_hold_time, double target_landing_height,
             double target_detection_size,
             
             // Home alignment PID parameters (8 parameters)  
             float home_align_lateral_kp, float home_align_lateral_ki, float home_align_lateral_kd,
             float home_align_vertical_kp, float home_align_vertical_ki, float home_align_vertical_kd,
             float home_align_distance_kp, float home_align_distance_ki, float home_align_distance_kd,
             float home_align_yaw_kp, float home_align_yaw_ki, float home_align_yaw_kd,
             
             double landing_velocity, double final_descent_height, double landing_timeout) 
        : fsm::FSM({"ERROR", "FINISHED"}) {

        Drone* drone = new Drone();
        this->blackboard_set<Drone>("drone", drone);

        drone->printSubscriptionStats();
        
        // MISSION PARAMETERS
        this->blackboard_set<double>("takeoff_height", takeoff_height);
        this->blackboard_set<double>("line_following_height", line_following_height);
        this->blackboard_set<double>("forward_velocity", forward_velocity);
        this->blackboard_set<double>("max_lateral_velocity", max_lateral_velocity);
        this->blackboard_set<double>("max_yaw_rate", max_yaw_rate);
        this->blackboard_set<double>("line_confidence_threshold", line_confidence_threshold);
        this->blackboard_set<double>("hose_confidence_threshold", hose_confidence_threshold);
        this->blackboard_set<double>("max_line_following_time", max_line_following_time);
        this->blackboard_set<double>("hose_detection_timeout", hose_detection_timeout);
        
        // LINE FOLLOWING PID PARAMETERS
        this->blackboard_set<float>("line_follow_lateral_kp", line_follow_lateral_kp);
        this->blackboard_set<float>("line_follow_lateral_ki", line_follow_lateral_ki);
        this->blackboard_set<float>("line_follow_lateral_kd", line_follow_lateral_kd);
        this->blackboard_set<float>("line_follow_height_kp", line_follow_height_kp);
        this->blackboard_set<float>("line_follow_height_ki", line_follow_height_ki);
        this->blackboard_set<float>("line_follow_height_kd", line_follow_height_kd);
        this->blackboard_set<float>("line_follow_yaw_kp", line_follow_yaw_kp);
        this->blackboard_set<float>("line_follow_yaw_ki", line_follow_yaw_ki);
        this->blackboard_set<float>("line_follow_yaw_kd", line_follow_yaw_kd);
        
        // HOSE ALIGNMENT PARAMETERS
        this->blackboard_set<double>("hose_alignment_tolerance", hose_alignment_tolerance);
        this->blackboard_set<double>("hose_alignment_velocity", hose_alignment_velocity);
        this->blackboard_set<double>("hose_stability_threshold", hose_stability_threshold);
        this->blackboard_set<double>("hose_alignment_timeout", hose_alignment_timeout);
        this->blackboard_set<double>("hose_stable_hold_time", hose_stable_hold_time);
        this->blackboard_set<double>("payload_drop_height", payload_drop_height);
        
        // HOSE ALIGNMENT PID PARAMETERS
        this->blackboard_set<float>("hose_align_lateral_kp", hose_align_lateral_kp);
        this->blackboard_set<float>("hose_align_lateral_ki", hose_align_lateral_ki);
        this->blackboard_set<float>("hose_align_lateral_kd", hose_align_lateral_kd);
        this->blackboard_set<float>("hose_align_vertical_kp", hose_align_vertical_kp);
        this->blackboard_set<float>("hose_align_vertical_ki", hose_align_vertical_ki);
        this->blackboard_set<float>("hose_align_vertical_kd", hose_align_vertical_kd);
        this->blackboard_set<float>("hose_align_yaw_kp", hose_align_yaw_kp);
        this->blackboard_set<float>("hose_align_yaw_ki", hose_align_yaw_ki);
        this->blackboard_set<float>("hose_align_yaw_kd", hose_align_yaw_kd);
        
        // RETURN HOME PARAMETERS
        this->blackboard_set<double>("max_horizontal_velocity", max_horizontal_velocity);
        this->blackboard_set<double>("home_position_tolerance", home_position_tolerance);
        this->blackboard_set<double>("return_height", return_height);
        this->blackboard_set<double>("return_navigation_timeout", return_navigation_timeout);
        
        // HOME ALIGNMENT PARAMETERS
        this->blackboard_set<double>("home_alignment_tolerance", home_alignment_tolerance);
        this->blackboard_set<double>("home_alignment_velocity", home_alignment_velocity);
        this->blackboard_set<double>("home_stability_threshold", home_stability_threshold);
        this->blackboard_set<double>("home_alignment_timeout", home_alignment_timeout);
        this->blackboard_set<double>("home_stable_hold_time", home_stable_hold_time);
        this->blackboard_set<double>("target_landing_height", target_landing_height);
        this->blackboard_set<double>("target_detection_size", target_detection_size);
        
        // HOME ALIGNMENT PID PARAMETERS
        this->blackboard_set<float>("home_align_lateral_kp", home_align_lateral_kp);
        this->blackboard_set<float>("home_align_lateral_ki", home_align_lateral_ki);
        this->blackboard_set<float>("home_align_lateral_kd", home_align_lateral_kd);
        this->blackboard_set<float>("home_align_vertical_kp", home_align_vertical_kp);
        this->blackboard_set<float>("home_align_vertical_ki", home_align_vertical_ki);
        this->blackboard_set<float>("home_align_vertical_kd", home_align_vertical_kd);
        this->blackboard_set<float>("home_align_distance_kp", home_align_distance_kp);
        this->blackboard_set<float>("home_align_distance_ki", home_align_distance_ki);
        this->blackboard_set<float>("home_align_distance_kd", home_align_distance_kd);
        this->blackboard_set<float>("home_align_yaw_kp", home_align_yaw_kp);
        this->blackboard_set<float>("home_align_yaw_ki", home_align_yaw_ki);
        this->blackboard_set<float>("home_align_yaw_kd", home_align_yaw_kd);
        
        // LANDING PARAMETERS
        this->blackboard_set<double>("landing_velocity", landing_velocity);
        this->blackboard_set<double>("final_descent_height", final_descent_height);
        this->blackboard_set<double>("landing_timeout", landing_timeout);

        // STATES
        this->add_state("INITIAL_TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("GO_TO_HOSE", std::make_unique<GoToHoseState>());
        this->add_state("ALIGN_HOSE", std::make_unique<AlignHoseState>());
        this->add_state("RETURN_HOME", std::make_unique<ReturnHomeState>());
        this->add_state("ALIGN_HOME", std::make_unique<AlignHomeState>());
        this->add_state("FINAL_LANDING", std::make_unique<LandingState>());

        // TRANSITIONS
        
        // Initial Takeoff transitions
        this->add_transitions("INITIAL_TAKEOFF", {
            {"TAKEOFF_COMPLETED", "GO_TO_HOSE"},
            {"TIMEOUT", "ERROR"}
        });

        // Go To Hose (line following) transitions
        this->add_transitions("GO_TO_HOSE", {
            {"HOSE_DETECTED", "ALIGN_HOSE"},
            {"TIMEOUT", "RETURN_HOME"}  // Fallback if hose not found
        });

        // Align Hose transitions
        this->add_transitions("ALIGN_HOSE", {
            {"ALIGNED", "RETURN_HOME"},
            {"TIMEOUT", "RETURN_HOME"}  // Fallback if alignment fails
        });

        // Return Home transitions
        this->add_transitions("RETURN_HOME", {
            {"ARRIVED_HOME", "ALIGN_HOME"},
            {"TIMEOUT", "FINAL_LANDING"}  // Emergency landing if return fails
        });

        // Align Home transitions
        this->add_transitions("ALIGN_HOME", {
            {"ALIGNED_HOME", "FINAL_LANDING"},
            {"TIMEOUT", "FINAL_LANDING"}  // Proceed to landing anyway
        });

        // Final Landing transitions
        this->add_transitions("FINAL_LANDING", {
            {"LANDED", "FINISHED"},
            {"TIMEOUT", "FINISHED"}  // Mission ends regardless
        });
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase2_node") {
        // Declare mission parameters
        this->declare_parameter("takeoff_height", -3.0);
        this->declare_parameter("line_following_height", -2.5);
        
        // Line following parameters
        this->declare_parameter("forward_velocity", 0.8);
        this->declare_parameter("max_lateral_velocity", 0.6);
        this->declare_parameter("max_yaw_rate", 0.5);
        this->declare_parameter("line_confidence_threshold", 0.5);
        this->declare_parameter("hose_confidence_threshold", 0.4);
        this->declare_parameter("max_line_following_time", 120.0);
        this->declare_parameter("hose_detection_timeout", 3.0);
        
        // Line following PID parameters (6 parameters)
        this->declare_parameter("line_follow_lateral_kp", 1.2);
        this->declare_parameter("line_follow_lateral_ki", 0.1);
        this->declare_parameter("line_follow_lateral_kd", 0.3);
        this->declare_parameter("line_follow_height_kp", 0.8);
        this->declare_parameter("line_follow_height_ki", 0.02);
        this->declare_parameter("line_follow_height_kd", 0.1);
        this->declare_parameter("line_follow_yaw_kp", 1.0);
        this->declare_parameter("line_follow_yaw_ki", 0.05);
        this->declare_parameter("line_follow_yaw_kd", 0.2);
        
        // Hose alignment parameters
        this->declare_parameter("hose_alignment_tolerance", 0.05);
        this->declare_parameter("hose_alignment_velocity", 0.3);
        this->declare_parameter("hose_stability_threshold", 0.02);
        this->declare_parameter("hose_alignment_timeout", 30.0);
        this->declare_parameter("hose_stable_hold_time", 2.0);
        this->declare_parameter("payload_drop_height", -1.5);
        
        // Hose alignment PID parameters (6 parameters)
        this->declare_parameter("hose_align_lateral_kp", 1.5);
        this->declare_parameter("hose_align_lateral_ki", 0.1);
        this->declare_parameter("hose_align_lateral_kd", 0.4);
        this->declare_parameter("hose_align_vertical_kp", 1.2);
        this->declare_parameter("hose_align_vertical_ki", 0.1);
        this->declare_parameter("hose_align_vertical_kd", 0.3);
        this->declare_parameter("hose_align_yaw_kp", 0.8);
        this->declare_parameter("hose_align_yaw_ki", 0.0);
        this->declare_parameter("hose_align_yaw_kd", 0.2);
        
        // Return home parameters
        this->declare_parameter("max_horizontal_velocity", 1.0);
        this->declare_parameter("home_position_tolerance", 0.3);
        this->declare_parameter("return_height", -2.0);
        this->declare_parameter("return_navigation_timeout", 60.0);
        
        // Home alignment parameters
        this->declare_parameter("home_alignment_tolerance", 0.03);
        this->declare_parameter("home_alignment_velocity", 0.2);
        this->declare_parameter("home_stability_threshold", 0.015);
        this->declare_parameter("home_alignment_timeout", 45.0);
        this->declare_parameter("home_stable_hold_time", 3.0);
        this->declare_parameter("target_landing_height", -0.8);
        this->declare_parameter("target_detection_size", 0.15);
        
        // Home alignment PID parameters (8 parameters)
        this->declare_parameter("home_align_lateral_kp", 1.5);
        this->declare_parameter("home_align_lateral_ki", 0.1);
        this->declare_parameter("home_align_lateral_kd", 0.4);
        this->declare_parameter("home_align_vertical_kp", 1.2);
        this->declare_parameter("home_align_vertical_ki", 0.1);
        this->declare_parameter("home_align_vertical_kd", 0.3);
        this->declare_parameter("home_align_distance_kp", 0.8);
        this->declare_parameter("home_align_distance_ki", 0.1);
        this->declare_parameter("home_align_distance_kd", 0.2);
        this->declare_parameter("home_align_yaw_kp", 0.8);
        this->declare_parameter("home_align_yaw_ki", 0.0);
        this->declare_parameter("home_align_yaw_kd", 0.2);
        
        // Landing parameters
        this->declare_parameter("landing_velocity", 0.3);
        this->declare_parameter("final_descent_height", -0.5);
        this->declare_parameter("landing_timeout", 30.0);

        // Get all parameter values
        double takeoff_height = this->get_parameter("takeoff_height").as_double();
        double line_following_height = this->get_parameter("line_following_height").as_double();
        double forward_velocity = this->get_parameter("forward_velocity").as_double();
        double max_lateral_velocity = this->get_parameter("max_lateral_velocity").as_double();
        double max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
        double line_confidence_threshold = this->get_parameter("line_confidence_threshold").as_double();
        double hose_confidence_threshold = this->get_parameter("hose_confidence_threshold").as_double();
        double max_line_following_time = this->get_parameter("max_line_following_time").as_double();
        double hose_detection_timeout = this->get_parameter("hose_detection_timeout").as_double();
        
        // Line following PID parameters
        float line_follow_lateral_kp = this->get_parameter("line_follow_lateral_kp").as_double();
        float line_follow_lateral_ki = this->get_parameter("line_follow_lateral_ki").as_double();
        float line_follow_lateral_kd = this->get_parameter("line_follow_lateral_kd").as_double();
        float line_follow_height_kp = this->get_parameter("line_follow_height_kp").as_double();
        float line_follow_height_ki = this->get_parameter("line_follow_height_ki").as_double();
        float line_follow_height_kd = this->get_parameter("line_follow_height_kd").as_double();
        float line_follow_yaw_kp = this->get_parameter("line_follow_yaw_kp").as_double();
        float line_follow_yaw_ki = this->get_parameter("line_follow_yaw_ki").as_double();
        float line_follow_yaw_kd = this->get_parameter("line_follow_yaw_kd").as_double();
        
        double hose_alignment_tolerance = this->get_parameter("hose_alignment_tolerance").as_double();
        double hose_alignment_velocity = this->get_parameter("hose_alignment_velocity").as_double();
        double hose_stability_threshold = this->get_parameter("hose_stability_threshold").as_double();
        double hose_alignment_timeout = this->get_parameter("hose_alignment_timeout").as_double();
        double hose_stable_hold_time = this->get_parameter("hose_stable_hold_time").as_double();
        double payload_drop_height = this->get_parameter("payload_drop_height").as_double();
        
        // Hose alignment PID parameters
        float hose_align_lateral_kp = this->get_parameter("hose_align_lateral_kp").as_double();
        float hose_align_lateral_ki = this->get_parameter("hose_align_lateral_ki").as_double();
        float hose_align_lateral_kd = this->get_parameter("hose_align_lateral_kd").as_double();
        float hose_align_vertical_kp = this->get_parameter("hose_align_vertical_kp").as_double();
        float hose_align_vertical_ki = this->get_parameter("hose_align_vertical_ki").as_double();
        float hose_align_vertical_kd = this->get_parameter("hose_align_vertical_kd").as_double();
        float hose_align_yaw_kp = this->get_parameter("hose_align_yaw_kp").as_double();
        float hose_align_yaw_ki = this->get_parameter("hose_align_yaw_ki").as_double();
        float hose_align_yaw_kd = this->get_parameter("hose_align_yaw_kd").as_double();
        double max_horizontal_velocity = this->get_parameter("max_horizontal_velocity").as_double();
        double home_position_tolerance = this->get_parameter("home_position_tolerance").as_double();
        double return_height = this->get_parameter("return_height").as_double();
        double return_navigation_timeout = this->get_parameter("return_navigation_timeout").as_double();
        double home_alignment_tolerance = this->get_parameter("home_alignment_tolerance").as_double();
        double home_alignment_velocity = this->get_parameter("home_alignment_velocity").as_double();
        double home_stability_threshold = this->get_parameter("home_stability_threshold").as_double();
        double home_alignment_timeout = this->get_parameter("home_alignment_timeout").as_double();
        double home_stable_hold_time = this->get_parameter("home_stable_hold_time").as_double();
        double target_landing_height = this->get_parameter("target_landing_height").as_double();
        double target_detection_size = this->get_parameter("target_detection_size").as_double();
        
        // Home alignment PID parameters
        float home_align_lateral_kp = this->get_parameter("home_align_lateral_kp").as_double();
        float home_align_lateral_ki = this->get_parameter("home_align_lateral_ki").as_double();
        float home_align_lateral_kd = this->get_parameter("home_align_lateral_kd").as_double();
        float home_align_vertical_kp = this->get_parameter("home_align_vertical_kp").as_double();
        float home_align_vertical_ki = this->get_parameter("home_align_vertical_ki").as_double();
        float home_align_vertical_kd = this->get_parameter("home_align_vertical_kd").as_double();
        float home_align_distance_kp = this->get_parameter("home_align_distance_kp").as_double();
        float home_align_distance_ki = this->get_parameter("home_align_distance_ki").as_double();
        float home_align_distance_kd = this->get_parameter("home_align_distance_kd").as_double();
        float home_align_yaw_kp = this->get_parameter("home_align_yaw_kp").as_double();
        float home_align_yaw_ki = this->get_parameter("home_align_yaw_ki").as_double();
        float home_align_yaw_kd = this->get_parameter("home_align_yaw_kd").as_double();
        
        double landing_velocity = this->get_parameter("landing_velocity").as_double();
        double final_descent_height = this->get_parameter("final_descent_height").as_double();
        double landing_timeout = this->get_parameter("landing_timeout").as_double();
        
        // Initialize the FSM with all parameters
        my_fsm = std::make_unique<Fase2FSM>(
            takeoff_height, line_following_height, forward_velocity, max_lateral_velocity, max_yaw_rate,
            line_confidence_threshold, hose_confidence_threshold, max_line_following_time, hose_detection_timeout,
            
            // Line following PID parameters
            line_follow_lateral_kp, line_follow_lateral_ki, line_follow_lateral_kd,
            line_follow_height_kp, line_follow_height_ki, line_follow_height_kd,
            line_follow_yaw_kp, line_follow_yaw_ki, line_follow_yaw_kd,
            
            hose_alignment_tolerance, hose_alignment_velocity, hose_stability_threshold, hose_alignment_timeout,
            hose_stable_hold_time, payload_drop_height,
            
            // Hose alignment PID parameters
            hose_align_lateral_kp, hose_align_lateral_ki, hose_align_lateral_kd,
            hose_align_vertical_kp, hose_align_vertical_ki, hose_align_vertical_kd,
            hose_align_yaw_kp, hose_align_yaw_ki, hose_align_yaw_kd,
            
            max_horizontal_velocity, home_position_tolerance, return_height, return_navigation_timeout,
            home_alignment_tolerance, home_alignment_velocity, home_stability_threshold, home_alignment_timeout,
            home_stable_hold_time, target_landing_height, target_detection_size,
            
            // Home alignment PID parameters
            home_align_lateral_kp, home_align_lateral_ki, home_align_lateral_kd,
            home_align_vertical_kp, home_align_vertical_ki, home_align_vertical_kd,
            home_align_distance_kp, home_align_distance_ki, home_align_distance_kd,
            home_align_yaw_kp, home_align_yaw_ki, home_align_yaw_kd,
            
            landing_velocity, final_descent_height, landing_timeout
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
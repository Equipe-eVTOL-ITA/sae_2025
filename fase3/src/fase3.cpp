#include <memory>
#include <iostream>
#include <vector>

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>

// imports dos estados
#include "fase3/initial_takeoff_state.hpp"
#include "fase3/search_base_state.hpp"
#include "fase3/goto_base_state.hpp"
#include "fase3/precision_align_state.hpp"
#include "fase3/landing_state.hpp"

class BouncingFSM : public fsm::FSM {
public:
    BouncingFSM(
        float takeoff_height, float max_vertical_velocity, float max_horizontal_velocity,
        float max_search_time,
        float pid_pos_kp, float pid_pos_ki, float pid_pos_kd,
        float setpoint,
        std::string class_id,
        float position_tolerance,
        float reso_x, float reso_y,
        float square_side_length
    ) : fsm::FSM({"ERROR", "FINISHED"}){
        
        Drone* drone = new Drone();
        
        this->blackboard_set<Drone>("drone", drone);

        const Eigen::Vector3d orientation = drone->getOrientation();

        std::vector<Base> bases;
        bases.push_back({drone->getLocalPosition(), true});

        // Parametros principais
        this->blackboard_set<float>("takeoff_height", takeoff_height);
        this->blackboard_set<float>("max_vertical_velocity", max_vertical_velocity);
        this->blackboard_set<float>("max_horizontal_velocity", max_horizontal_velocity);
        this->blackboard_set<float>("initial_yaw", orientation[2]);
        this->blackboard_set<float>("resolution_x", reso_x);
        this->blackboard_set<float>("resolution_y", reso_y);
    
        // Parametros da missao
        this->blackboard_set<float>("max_search_time", max_search_time);
        this->blackboard_set<std::string>("class_id", class_id);
        this->blackboard_set<float>("square_side_length", square_side_length);
        this->blackboard_set<float>("position_tolerance", position_tolerance);
        this->blackboard_set<std::vector<Base>>("bases", bases);
        this->blackboard_set<bool>("finished_bases", false);
        this->blackboard_set<float>("setpoint", setpoint);

        // Pontos da Aerena
        /*
        E---A---B
        |       |
        |       |
        D-------C
        */
        std::vector<ArenaPoint> waypoints;
        float l2 = square_side_length/2.0;
        waypoints.push_back({
            Eigen::Vector3d({l2, 0, takeoff_height}) // A
        });
        waypoints.push_back({
            Eigen::Vector3d({l2, l2, takeoff_height}) // B
        });
        waypoints.push_back({
            Eigen::Vector3d({-l2, l2, takeoff_height}) // C
        });
        waypoints.push_back({
            Eigen::Vector3d({-l2, -l2, takeoff_height}) // D
        });
        waypoints.push_back({
            Eigen::Vector3d({l2, -l2, takeoff_height}) // E
        });
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);

        // Parametos do PID da posicao
        this->blackboard_set<float>("pid_pos_kp", pid_pos_kp);
        this->blackboard_set<float>("pid_pos_ki", pid_pos_ki);
        this->blackboard_set<float>("pid_pos_kd", pid_pos_kd);

        // Estados
        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH BASE", std::make_unique<SearchBaseState>());
        this->add_state("GO TO BASE", std::make_unique<GoToBaseState>());
        this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
        this->add_state("PRECISION LANDING", std::make_unique<LandingState>());

        // Definicao das transicoes
        /*
        Funcionam como grafos:
            no inicial, {condicao para a transicao, no final}
        */
        this->add_transitions("INITIAL TAKEOFF", {
            {"INITIAL TAKEOFF COMPLETED", "SEARCH BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH BASE", {
            {"BASE FOUND", "GO TO BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GO TO BASE", {
            {"OVER THE BASE", "PRECISION ALIGN"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PRECISION ALIGN", {
            {"PRECISELY ALIGNED", "PRECISION LANDING"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PRECISION LANDING", {
            {"LANDED", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });
    }

};


class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase3_fsm") {
        // Parametros principais
        this->declare_parameter("takeoff_height", -2.0);
        this->declare_parameter("max_vertical_velocity", 1.5);
        this->declare_parameter("max_horizontal_velocity", 1.0);
        this->declare_parameter("resolution_x", 800.0);
        this->declare_parameter("resolution_y", 800.0);
        this->declare_parameter("square_side_length", 2.0);

        // Parametros da missao
        this->declare_parameter("max_search_time", 30.0);
        this->declare_parameter("class_id", "");
        this->declare_parameter("position_tolerance", 0.08);

        // Parametros do PID
        this->declare_parameter("pid_pos_kp", 0.9);
        this->declare_parameter("pid_pos_ki", 0.0);
        this->declare_parameter("pid_pos_kd", 0.05);
        this->declare_parameter("setpoint", 0.5);

        // Obtendo os valores dos parametros...

        // ... principais
        float takeoff_height = this->get_parameter("takeoff_height").as_double();
        float max_vertical_velocity = this->get_parameter("max_vertical_velocity").as_double();
        float max_horizontal_velocity = this->get_parameter("max_horizontal_velocity").as_double();
        float reso_x = this->get_parameter("resolution_x").as_double();
        float reso_y = this->get_parameter("resolution_y").as_double();

        // ... da missao
        float max_search_time = this->get_parameter("max_search_time").as_double();
        std::string class_id = this->get_parameter("class_id").as_string();
        float square_side_length = this->get_parameter("square_side_length").as_double();
        
        // ... do PID
        float pid_pos_kp = this->get_parameter("pid_pos_kp").as_double();
        float pid_pos_ki = this->get_parameter("pid_pos_ki").as_double();
        float pid_pos_kd = this->get_parameter("pid_pos_kd").as_double();
        float setpoint = this->get_parameter("setpoint").as_double();
        float position_tolerance = this->get_parameter("position_tolerance").as_double();

        // Inicializando um ponteiro inteligente para uma instancia do Finite State Machine
        b_fsm = std::make_unique<BouncingFSM>(
            takeoff_height, max_vertical_velocity, max_horizontal_velocity, max_search_time,
            pid_pos_kp, pid_pos_ki, pid_pos_kd,
            setpoint,
            class_id,
            position_tolerance,
            reso_x, reso_y,
            square_side_length
        );

        // Wall timer para executar tarefas com base na passagem do tempo real
        // Garante que as acoes baseadas em tempo sejam consistentes e previsiveis
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // Faz rodar em aproximadamente 20 Hz
            std::bind(&NodeFSM::executeFSM, this) // seta um callback para o timer do ROS2 para ser chamado periodicamente
        );

    }

    void executeFSM() {
        if (rclcpp::ok() && !b_fsm->is_finished()) {
            b_fsm->execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    std::unique_ptr<BouncingFSM> b_fsm; // Declaracao do ponteiro inteligente para uma instancia do FSM
    rclcpp::TimerBase::SharedPtr timer_; // Declaracao do wall timer para garantir boa passagem de tempo
};


int main(int argc, const char *argv[]){
    rclcpp::init(argc, argv);

    auto node_bouncing = std::make_shared<NodeFSM>();
    rclcpp::spin(node_bouncing);

    return 0;
}
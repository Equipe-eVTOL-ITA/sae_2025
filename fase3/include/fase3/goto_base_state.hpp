#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = blackboard.get<Drone>("drone");
        if(this->drone == nullptr) return;
        this->drone->log("STATE: Voando para a base ja detectada...");

        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        float takeoff_height = *blackboard.get<float>("takeoff_height");
        
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        
        // Lendo o que esta escrito no "Quadro negro" o que o search_base_state escreveu
        // no caso, a estimativa da posicao da base no solo
        const Eigen::Vector2d estimated_base_position = *blackboard.get<Eigen::Vector2d>("estimated_base_position_on_ground");
        // como nao queremos ir direto ja pousando, o nosso goal eh o x e y dessa posicao estimada
        // porem, a altura nao eh 0, mas sim a altura de takeoff. Entao:
        this->goal = Eigen::Vector3d(
            estimated_base_position.x(),
            estimated_base_position.y(),
            takeoff_height
        );

        yaw = this->drone->getOrientation()[2];
    }


    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        this->pos = this->drone->getLocalPosition();

        if((this->pos - this->goal).norm() < this->position_tolerance){
            return "OVER THE BASE";
        }

        Eigen::Vector3d diff = this->goal - this->pos;
        Eigen::Vector3d little_goal = this->pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);

        this->drone->setLocalPosition(
            little_goal[0],
            little_goal[1],
            little_goal[2],
            yaw
        );

        return "";        
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    
    float max_velocity;
    float yaw;
    float position_tolerance;

    Drone* drone;
};
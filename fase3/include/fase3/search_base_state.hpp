#include <cmath>
#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: Euler Spiral");

        r = *blackboard.get<float>("spiral_radius_initial");
        r_limit = *blackboard.get<float>("spiral_radius_limit");
        max_velocity = *blackboard.get<float>("max_horizontal_velocity");

        center = drone->getLocalPosition();
        z_ref = center.z();
        yaw_ref = drone->getOrientation().z();
        
        angular_vel = max_velocity / r;
        theta = 0.0;
        dt = 0.05;

        double tempo_busca = 1.5 * 60 / dt; // 1 minuto e meio
        step = (r_limit - r) / tempo_busca;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        r += step;
        angular_vel = max_velocity / r;
        theta += angular_vel * dt;
        Eigen::Vector3d p = center;
        p.x() += r * std::cos(theta);
        p.y() += r * std::sin(theta);
        p.z()  = z_ref;
        drone->setLocalPosition(p.x(), p.y(), p.z(), yaw_ref);

        if (r > r_limit) return "FOUND BASE";

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Eigen::Vector3d pos = drone->getLocalPosition();
        blackboard.set<Eigen::Vector2d>("approximate_base", Eigen::Vector2d(pos.x(), pos.y()));
        drone->log("Approximate base: " + std::to_string(pos.x()) + " " + std::to_string(pos.y()));
    }

private:
    Drone* drone{nullptr};
    Eigen::Vector3d center;
    double z_ref, yaw_ref, dt;
    double step, r, r_limit;
    double theta, angular_vel, max_velocity;
};
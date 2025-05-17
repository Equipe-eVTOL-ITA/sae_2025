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

        max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        // Record the starting position and use its z coordinate.
        pos0 = drone->getLocalPosition();
        pos0[2] = takeoff_height;
        current_yaw = drone->getOrientation()[2];

        // initialize spiral state:
        s = 0.0;
        dt = 0.05;        // time step in seconds
        spiral_a = 0.3;  // parameter controlling rate of curvature increase
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        // Get the current position.
        Eigen::Vector3d current_pos = drone->getLocalPosition();

        // Compute the arc length increment based on constant dt.
        double ds = max_velocity * dt * 10;
        s += ds;
        // Increase yaw using the instantaneous curvature: k(s)= spiral_a * s.
        current_yaw += spiral_a * s * ds;
        
        // Calculate the new position incrementally:
        Eigen::Vector3d new_pos = current_pos;
        new_pos[0] += ds * std::cos(current_yaw);
        new_pos[1] += ds * std::sin(current_yaw);
        // Maintain constant altitude.
        new_pos[2] = pos0[2];

        drone->setLocalPosition(new_pos[0], new_pos[1], new_pos[2], current_yaw);

        // Terminate the state if the spiral arc-length exceeds a threshold.
        if (s > 20.0) {
            return "FOUND BASE";
        }
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        blackboard.set<Eigen::Vector2d>("approximate_base", Eigen::Vector2d(pos0[0], pos0[1]));
    }

private:
    Drone* drone;
    float max_velocity;
    // Variables for Euler spiral trajectory:
    Eigen::Vector3d pos0;
    double s;          // accumulated arc length
    double dt;         // time step
    double spiral_a;   // curvature parameter, k(s) = spiral_a * s
    double current_yaw;
};
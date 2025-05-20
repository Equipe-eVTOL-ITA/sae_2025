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
        class_id = *blackboard.get<std::string>("classe_base_pouso");

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

        Eigen::Vector3d pos = drone->getLocalPosition();
        float yaw = drone->getOrientation().z();

        auto bboxes = drone->getVerticalBboxes();

        if (!bboxes.empty()) {
            for (const auto& bbox : bboxes) {
                if (bbox.class_id == class_id) {
                    const Eigen::Vector2d approx_base = getApproximateBase(bbox.center_x, bbox.center_y, pos, yaw);
                    blackboard.set<Eigen::Vector2d>("approximate_base", approx_base);
                    drone->log("Estimate: {" + std::to_string(approx_base.x()) +
                                ", " + std::to_string(approx_base.y()) + "}");
                    return "FOUND BASE";
                }
            }
        }


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
    std::string class_id;

    Eigen::Vector2d getApproximateBase(double x, double y, Eigen::Vector3d pos, double yaw) {
        // IMAGE: x to the right, y to the bottom
        // DRONE: x to the front, y to the right

        // IMAGE TRANSFORMS -------------------
        
        double x_max_dist = 3.0; //distance on ground from left to right of picture when flying at takeoff altitude
        double y_max_dist = x_max_dist; //based on 4:3 image proportion - from up to down

        double height = -pos.z();

        //Considering that pixel dimensions are like angular resolution
        double k_x = std::atan((x_max_dist / 2) / height);
        double k_y = std::atan((y_max_dist / 2) / height);


        // DRONE TRANSFORMS -------------------

        // Left side: drone; right side: image
        double rotated_distance_y = height * std::tan(k_x * 2 * (x-0.5));
        double rotated_distance_x = - height * std::tan(k_y * 2 * (y-0.5)); // minus sign due to the image coordinate system

        double distance_x = rotated_distance_x * std::cos(yaw) - rotated_distance_y * std::sin(yaw);
        double distance_y = rotated_distance_x * std::sin(yaw) + rotated_distance_y * std::cos(yaw);

        distance_x = pos.x() + rotated_distance_x;
        distance_y = pos.y() + rotated_distance_y;
        
        return Eigen::Vector2d({distance_x, distance_y});
    }
};
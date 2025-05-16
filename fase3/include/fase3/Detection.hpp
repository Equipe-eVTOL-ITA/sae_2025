#pragma once

#include "drone/Drone.hpp"

class Detection{
public:
    Detection(std::vector<DronePX4::BoundingBox> bboxes){
        this->computeBboxes(bboxes);
    }

    DronePX4::BoundingBox getClosestBbox(){
        return this->closest_bbox_;
    }
    
    float getMinDistance(){
        return this->min_distance_;
    }

    bool isThereDetection(){
        return this->is_there_detection_;
    }

private:
    DronePX4::BoundingBox closest_bbox_;
    float min_distance_{0.0};
    bool is_there_detection_{false};

    void computeBboxes(std::vector<DronePX4::BoundingBox> bboxes){
        Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});

        if (!bboxes.empty()) {
            this->is_there_detection_ = true;
            float min_distance = 2.0;
            DronePX4::BoundingBox closest_bbox;
            //Find closest bbox to center
            for (const auto& bbox : bboxes) {
                double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                }                    
            }
            this->closest_bbox_ = closest_bbox;
            this->min_distance_ = min_distance;
        }
        else{
            this->is_there_detection_ = false;
        }
    }

};
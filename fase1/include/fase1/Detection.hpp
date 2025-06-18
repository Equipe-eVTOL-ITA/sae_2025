#pragma once

#include "drone/Drone.hpp"

class Detection{
public:
    Detection(std::vector<DronePX4::BoundingBox> bboxes){
        this->computeBboxes(bboxes);
    }
    
    // Constructor with color filtering
    Detection(std::vector<DronePX4::BoundingBox> bboxes, const std::string& target_color){
        this->computeBboxes(bboxes, target_color);
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
        computeBboxes(bboxes, ""); // No color filtering
    }
    
    void computeBboxes(std::vector<DronePX4::BoundingBox> bboxes, const std::string& target_color){
        Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});

        if (!bboxes.empty()) {
            float min_distance = 2.0;
            DronePX4::BoundingBox closest_bbox;
            bool found_target = false;
            
            //Find closest bbox to center, optionally filtering by color
            for (const auto& bbox : bboxes) {
                // If target_color is specified, filter by color
                if (!target_color.empty() && bbox.class_id != target_color) {
                    continue; // Skip this detection if it doesn't match target color
                }
                
                double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                    found_target = true;
                }                    
            }
            
            // Only set detection as true if we found the target color (or no color filtering)
            this->is_there_detection_ = found_target;
            if (found_target) {
                this->closest_bbox_ = closest_bbox;
                this->min_distance_ = min_distance;
            }
        }
        else{
            this->is_there_detection_ = false;
        }
    }

};
cmake_minimum_required(VERSION 3.5)
project(sae_cv_utils)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(barcode_detector
  custom_msgs
)

ament_package()

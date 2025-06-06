cmake_minimum_required(VERSION 3.8)
project(sae_drone_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



include_directories(include)

add_library(sae_drone_lib SHARED
  src/Drone.cpp
)

ament_target_dependencies(sae_drone_lib
  Eigen3
  px4_msgs
  rclcpp
  tf2
  cv_bridge
  vision_msgs
  custom_msgs
)

add_executable(pos_to_rviz
  src/pos_to_rviz.cpp
)

ament_target_dependencies(pos_to_rviz
  rclcpp
  geometry_msgs
  nav_msgs
  custom_msgs
)

ament_export_targets(sae_drone_lib HAS_LIBRARY_TARGET)

ament_export_dependencies(Eigen3 px4_msgs rclcpp tf2 cv_bridge vision_msgs custom_msgs)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/drone
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
 
install(
  TARGETS sae_drone_lib
  EXPORT sae_drone_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS pos_to_rviz
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

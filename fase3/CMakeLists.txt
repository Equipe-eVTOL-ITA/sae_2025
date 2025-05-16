cmake_minimum_required(VERSION 3.8)
project(sae_fase3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(sae_drone_lib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

#fase3

add_executable(fase3
  src/fase3.cpp
)

ament_target_dependencies(fase3
  rclcpp
  fsm
  sae_drone_lib
  Eigen3
  vision_msgs
)

target_link_libraries(fase3 ${OpenCV_LIBS})


target_link_libraries(fase3 ${OpenCV_LIBS})

#Install

install(
  TARGETS
    fase3
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

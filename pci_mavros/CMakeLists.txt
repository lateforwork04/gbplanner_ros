cmake_minimum_required(VERSION 3.8)
project(pci_mavros)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wno-sign-compare -Werror=return-type)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf2
  tf2_ros
  planner_common
  planner_control_interface
)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_executable(pci_mavros_node src/pci_mavros.cpp src/pci_mavros_node.cpp)
target_include_directories(pci_mavros_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(pci_mavros_node ${dependencies})

install(TARGETS pci_mavros_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(planner_control_interface)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wno-sign-compare -Wno-catch-value -Werror=return-type)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interactive_markers
  planner_common
  planner_msgs
  planner_semantic_msgs
)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/planner_control_interface.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} Eigen3)
ament_package()

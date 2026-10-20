cmake_minimum_required(VERSION 3.8)
project(gbplanner_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(dependencies rclcpp rviz_common pluginlib std_srvs planner_msgs)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/gbplanner_ui.cpp src/gbplanner_ui.h)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

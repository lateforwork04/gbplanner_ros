cmake_minimum_required(VERSION 3.8)
project(map_manager)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-format -Werror=return-type)
add_definitions(-DRAY_CAST_METHOD=0)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# voxblox is built as static libraries that do not export their protobuf dependency.
find_package(Protobuf REQUIRED)
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
  voxblox
  voxblox_ros
  planner_common
  planner_msgs
  planner_semantic_msgs
)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/map_manager.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} protobuf::libprotobuf)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} Eigen3 PCL Protobuf)
ament_package()

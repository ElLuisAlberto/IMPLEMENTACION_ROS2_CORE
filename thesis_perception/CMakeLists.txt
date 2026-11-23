cmake_minimum_required(VERSION 3.8)
project(thesis_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters search segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(thesis_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pointcloud_preprocessor_node
  src/pointcloud_preprocessor_node.cpp
)
ament_target_dependencies(pointcloud_preprocessor_node
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_include_directories(pointcloud_preprocessor_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_preprocessor_node ${PCL_LIBRARIES})
target_compile_definitions(pointcloud_preprocessor_node PRIVATE ${PCL_DEFINITIONS})

add_executable(pointcloud_cluster_probe
  src/pointcloud_cluster_probe.cpp
)
ament_target_dependencies(pointcloud_cluster_probe
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_include_directories(pointcloud_cluster_probe PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_cluster_probe ${PCL_LIBRARIES})
target_compile_definitions(pointcloud_cluster_probe PRIVATE ${PCL_DEFINITIONS})

add_executable(obstacle_extractor_node
  src/obstacle_extractor_node.cpp
)
ament_target_dependencies(obstacle_extractor_node
  diagnostic_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  thesis_interfaces
  visualization_msgs
)
target_include_directories(obstacle_extractor_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(obstacle_extractor_node ${PCL_LIBRARIES})
target_compile_definitions(obstacle_extractor_node PRIVATE ${PCL_DEFINITIONS})

# Comparison implementation kept for reproducible historical experiments.
add_executable(cloud_prediction_node src/alternativas/cloud_prediction_node.cpp)
ament_target_dependencies(cloud_prediction_node
  diagnostic_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
target_include_directories(cloud_prediction_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_prediction_node ${PCL_LIBRARIES})
target_compile_definitions(cloud_prediction_node PRIVATE ${PCL_DEFINITIONS})

# Comparison implementation kept for the previous RAFT sweep.
add_executable(raft_sweep_node src/alternativas/raft_sweep_node.cpp)
ament_target_dependencies(raft_sweep_node rclcpp sensor_msgs visualization_msgs)
target_include_directories(raft_sweep_node PRIVATE include ${PCL_INCLUDE_DIRS})

# Current visualization implementation.
add_executable(raft_lines_node src/raft_lines_node.cpp)
ament_target_dependencies(raft_lines_node rclcpp sensor_msgs visualization_msgs)

add_executable(perception_health_node
  src/perception_health_node.cpp
)

add_executable(visualization_overlay_node
  src/visualization_overlay_node.cpp
)
ament_target_dependencies(visualization_overlay_node rclcpp sensor_msgs)
ament_target_dependencies(perception_health_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  pointcloud_preprocessor_node
  pointcloud_cluster_probe
  cloud_prediction_node
  raft_sweep_node
  raft_lines_node
  obstacle_extractor_node
  perception_health_node
  visualization_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python runtime scripts. Keep the shared base installed alongside both
# predictor entry points because the predictors import it by module name.
install(PROGRAMS
  scripts/distance_comparison_views.py
  scripts/point_flow_preview.py
  scripts/point_flow_raft_preview_experimental.py
  scripts/alternativas/point_flow_raft_preview.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_perception_safety_contract
    test/test_perception_safety_contract.py
  )
  ament_add_gtest(test_swept_voxel_grid test/test_swept_voxel_grid.cpp)
  target_include_directories(test_swept_voxel_grid PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_add_gtest(test_motion_estimator test/test_motion_estimator.cpp)
  target_include_directories(test_motion_estimator PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_add_gtest(test_stationary_voxels test/test_stationary_voxels.cpp)
  target_include_directories(test_stationary_voxels PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_add_gtest(test_pointcloud_cluster_metrics
    test/test_pointcloud_cluster_metrics.cpp
  )
  target_include_directories(test_pointcloud_cluster_metrics PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_pointcloud_cluster_metrics ${PCL_LIBRARIES})
  target_compile_definitions(test_pointcloud_cluster_metrics PRIVATE ${PCL_DEFINITIONS})
endif()

ament_package()

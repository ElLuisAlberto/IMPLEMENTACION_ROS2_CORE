# Copyright 2026 Luis Alberto Munoz Marin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch D435i acquisition and isolated perception without a robot."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build a camera-only launch safe for calibration and characterization."""
    camera_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])
    processing_launch = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "launch",
        "perception_processing.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "rviz",
        "d435i_perception.rviz",
    ])
    overlay_rviz_config = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "rviz",
        "d435i_jaco_experimental.rviz",
    ])
    overlay_parameters = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "config",
        "d435i_overlay_experimental.yaml",
    ])

    camera = GroupAction(
        actions=[
            SetParameter(
                name="pointcloud.pointcloud_qos",
                value="SENSOR_DATA",
            ),
            # The upstream launch does not expose these sensor options as
            # arguments. Pass them through the launch ROS parameter scope.
            SetParameter(name="depth_module.visual_preset", value=4),
            SetParameter(name="depth_module.laser_power", value=150.0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                condition=IfCondition(LaunchConfiguration("start_camera")),
                launch_arguments={
                    "camera_namespace": "camera",
                    "camera_name": "d435i",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_gyro": LaunchConfiguration("enable_imu"),
                    "enable_accel": LaunchConfiguration("enable_imu"),
                    "unite_imu_method": "2",
                    "gyro_fps": "200",
                    "accel_fps": "100",
                    "enable_sync": "true",
                    "pointcloud.enable": "true",
                    "pointcloud.stream_filter": "2",
                    "align_depth.enable": "true",
                    "depth_module.enable_auto_exposure": "false",
                    "depth_module.exposure": "10000",
                    "decimation_filter.enable": LaunchConfiguration(
                        "enable_decimation_filter"
                    ),
                    "decimation_filter.filter_magnitude": LaunchConfiguration(
                        "decimation_magnitude"
                    ),
                    "spatial_filter.enable": LaunchConfiguration(
                        "enable_spatial_filter"
                    ),
                    "temporal_filter.enable": LaunchConfiguration(
                        "enable_temporal_filter"
                    ),
                    "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                }.items(),
            ),
        ],
    )

    processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(processing_launch),
        condition=IfCondition(LaunchConfiguration("start_processing")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "target_frame": LaunchConfiguration("target_frame"),
            "restamp_output": LaunchConfiguration("restamp_output"),
            "enable_crop": LaunchConfiguration("enable_crop"),
            "enable_voxel": LaunchConfiguration("enable_voxel"),
            "outlier_method": LaunchConfiguration("outlier_method"),
            "outlier_mean_k": LaunchConfiguration("outlier_mean_k"),
            "outlier_stddev": LaunchConfiguration("outlier_stddev"),
            "outlier_radius_m": LaunchConfiguration("outlier_radius_m"),
            "outlier_min_neighbors": LaunchConfiguration("outlier_min_neighbors"),
            "voxel_leaf_size": LaunchConfiguration("voxel_leaf_size"),
            "min_valid_range_m": LaunchConfiguration(
                "min_valid_range_m"
            ),
            "max_valid_range_m": LaunchConfiguration(
                "max_valid_range_m"
            ),
            "min_x": LaunchConfiguration("min_x"),
            "max_x": LaunchConfiguration("max_x"),
            "min_y": LaunchConfiguration("min_y"),
            "max_y": LaunchConfiguration("max_y"),
            "min_z": LaunchConfiguration("min_z"),
            "max_z": LaunchConfiguration("max_z"),
            "start_extractor": LaunchConfiguration("start_extractor"),
            "obstacle_topic": LaunchConfiguration("obstacle_topic"),
            "imu_required": LaunchConfiguration("imu_required"),
            "publish_static_extrinsic": LaunchConfiguration(
                "publish_static_extrinsic"
            ),
            "extrinsic_parent_frame": LaunchConfiguration(
                "extrinsic_parent_frame"
            ),
            "camera_link_frame": LaunchConfiguration("camera_link_frame"),
            "extrinsic_x": LaunchConfiguration("extrinsic_x"),
            "extrinsic_y": LaunchConfiguration("extrinsic_y"),
            "extrinsic_z": LaunchConfiguration("extrinsic_z"),
            "extrinsic_roll": LaunchConfiguration("extrinsic_roll"),
            "extrinsic_pitch": LaunchConfiguration("extrinsic_pitch"),
            "extrinsic_yaw": LaunchConfiguration("extrinsic_yaw"),
        }.items(),
    )

    prediction = Node(
        package="thesis_perception", executable="cloud_prediction_node",
        name="cloud_prediction", output="screen",
        condition=IfCondition(LaunchConfiguration("start_cloud_prediction")),
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "max_valid_range_m": ParameterValue(
                LaunchConfiguration("max_valid_range_m"), value_type=float),
            "sweep_enabled": ParameterValue(
                LaunchConfiguration("sweep_enabled"), value_type=bool),
            "sweep_voxel_m": ParameterValue(
                LaunchConfiguration("sweep_voxel_m"), value_type=float),
            "sweep_max_cells": ParameterValue(
                LaunchConfiguration("sweep_max_cells"), value_type=int),
            "sweep_max_steps": ParameterValue(
                LaunchConfiguration("sweep_max_steps"), value_type=int),
        }],
    )

    distance_comparison = Node(
        package="thesis_perception",
        executable="distance_comparison_views.py",
        name="comparacion_distancia_suavizado",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("start_distance_comparison")
        ),
    )

    overlay = Node(
        package="thesis_perception",
        executable="visualization_overlay_node",
        name="visualization_overlay",
        parameters=[overlay_parameters],
        condition=IfCondition(LaunchConfiguration("show_robot_overlay")),
        output="screen",
    )

    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        actions=[
            SetEnvironmentVariable(
                "__NV_PRIME_RENDER_OFFLOAD", "1",
                condition=IfCondition(LaunchConfiguration("rviz_use_nvidia")),
            ),
            SetEnvironmentVariable(
                "__GLX_VENDOR_LIBRARY_NAME", "nvidia",
                condition=IfCondition(LaunchConfiguration("rviz_use_nvidia")),
            ),
            Node(
                package="rviz2", executable="rviz2",
                name="d435i_perception_rviz", output="screen",
                condition=UnlessCondition(LaunchConfiguration("show_robot_overlay")),
                arguments=["-d", rviz_config],
            ),
            Node(
                package="rviz2", executable="rviz2",
                name="d435i_experimental_overlay_rviz", output="screen",
                condition=IfCondition(LaunchConfiguration("show_robot_overlay")),
                arguments=["-d", overlay_rviz_config],
            ),
        ],
    )

    declarations = [
        DeclareLaunchArgument("depth_profile", default_value="640,480,60"),
        DeclareLaunchArgument("color_profile", default_value="640,480,60"),
        DeclareLaunchArgument("start_cloud_prediction", default_value="false"),
        DeclareLaunchArgument(
            "start_distance_comparison", default_value="false"
        ),
        DeclareLaunchArgument("sweep_enabled", default_value="true"),
        DeclareLaunchArgument("sweep_voxel_m", default_value="0.02"),
        DeclareLaunchArgument("sweep_max_cells", default_value="40000"),
        DeclareLaunchArgument("sweep_max_steps", default_value="64"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_processing", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_use_nvidia", default_value="true"),
        DeclareLaunchArgument("show_robot_overlay", default_value="false"),
        DeclareLaunchArgument(
            "enable_imu",
            default_value="false",
            description=(
                "Enable accelerometer and gyroscope only when inertial "
                "data is required and host IIO permissions are configured."
            ),
        ),
        DeclareLaunchArgument(
            "enable_decimation_filter",
            default_value="false",
            description="Reduce depth workload before creating PointCloud2.",
        ),
        DeclareLaunchArgument(
            "decimation_magnitude",
            default_value="2",
            description="Depth decimation factor; 2 preserves test detail.",
        ),
        DeclareLaunchArgument(
            "enable_spatial_filter",
            default_value="false",
            description="Stabilize local depth surfaces before pointcloud.",
        ),
        DeclareLaunchArgument(
            "enable_temporal_filter",
            default_value="false",
            description="Reduce frame-to-frame depth flicker for fixed camera.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "target_frame",
            default_value="",
            description="Preserve the source frame until extrinsic calibration.",
        ),
        DeclareLaunchArgument("restamp_output", default_value="false"),
        DeclareLaunchArgument("enable_crop", default_value="false"),
        DeclareLaunchArgument("enable_voxel", default_value="true"),
        DeclareLaunchArgument("outlier_method", default_value="none"),
        DeclareLaunchArgument("outlier_mean_k", default_value="12"),
        DeclareLaunchArgument("outlier_stddev", default_value="2.0"),
        DeclareLaunchArgument("outlier_radius_m", default_value="0.03"),
        DeclareLaunchArgument("outlier_min_neighbors", default_value="2"),
        DeclareLaunchArgument("voxel_leaf_size", default_value="0.01"),
        DeclareLaunchArgument("min_valid_range_m", default_value="0.10"),
        DeclareLaunchArgument("max_valid_range_m", default_value="1.2"),
        DeclareLaunchArgument("min_x", default_value="-1.5"),
        DeclareLaunchArgument("max_x", default_value="1.5"),
        DeclareLaunchArgument("min_y", default_value="-1.5"),
        DeclareLaunchArgument("max_y", default_value="1.5"),
        DeclareLaunchArgument("min_z", default_value="-1.0"),
        DeclareLaunchArgument("max_z", default_value="2.5"),
        DeclareLaunchArgument("start_extractor", default_value="true"),
        DeclareLaunchArgument("imu_required", default_value="false"),
        DeclareLaunchArgument(
            "obstacle_topic",
            default_value="/thesis/perception/obstacle_candidate",
        ),
        DeclareLaunchArgument(
            "publish_static_extrinsic", default_value="false"
        ),
        DeclareLaunchArgument(
            "extrinsic_parent_frame", default_value="world"
        ),
        DeclareLaunchArgument("camera_link_frame", default_value="d435i_link"),
        DeclareLaunchArgument("extrinsic_x", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_y", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_z", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_roll", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_pitch", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_yaw", default_value="0.0"),
    ]

    return LaunchDescription(
        declarations
        + [camera, processing, prediction, distance_comparison, overlay, rviz]
    )

"""
f1tenth_bringup/launch/f1tenth_stack.launch.py
================================================
Full F1Tenth autonomous stack with SLAM and diagnostics.

Nodes launched:
  1. zed_wrapper/zed_node        — camera driver
  2. vision_perception/line_mapper — lane detection (publishes raw points + raw guidance)
  3. vision_perception/lane_slam  — SLAM fusion (publishes fused guidance)
  4. vision_perception/slam_visualizer — BEV diagnostics (optional, disable for race)
  5. control_planning/kart_controller — PD steering
  6. vesc_ackermann/ackermann_to_vesc — Ackermann → VESC
  7. vesc_driver/vesc_driver_node     — serial to VESC
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('f1tenth_bringup')
    vesc_params = os.path.join(bringup_dir, 'config', 'vesc_params.yaml')

    speed_arg = DeclareLaunchArgument(
        'speed_nominal', default_value='1.0',
        description='Cruise speed in m/s')

    enable_viz_arg = DeclareLaunchArgument(
        'enable_visualizer', default_value='true',
        description='Enable SLAM BEV visualizer node')

    # -- 1. ZED Camera --------------------------------------------------------
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            'depth_mode': 'NEURAL',
            'point_cloud_freq': '15.0',
            'coordinate_system': 'RIGHT_HANDED_Z_UP_X_FWD',
        }.items(),
    )

    # -- 2. Perception (line_mapper — raw detection only) ---------------------
    perception_node = Node(
        package='vision_perception',
        executable='line_mapper',
        name='vision_perception_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 3. SLAM (lane_slam — fuses raw + odom → publishes /perception/lane_guidance)
    slam_node = Node(
        package='vision_perception',
        executable='lane_slam',
        name='lane_slam_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 4. Visualizer (optional — disable for race with enable_visualizer:=false)
    viz_node = Node(
        package='vision_perception',
        executable='slam_visualizer',
        name='slam_visualizer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visualizer')),
    )

    # -- 5. Control -----------------------------------------------------------
    control_node = Node(
        package='control_planning',
        executable='kart_controller',
        name='control_planning_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 6. VESC Ackermann Translator -----------------------------------------
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 7. VESC Driver -------------------------------------------------------
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[vesc_params],
    )

    return LaunchDescription([
        speed_arg,
        enable_viz_arg,
        zed_node,
        perception_node,
        slam_node,
        viz_node,
        control_node,
        ackermann_to_vesc_node,
        vesc_driver_node,
    ])

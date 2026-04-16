"""
f1tenth_bringup/launch/f1tenth_stack.launch.py
================================================
Full F1Tenth autonomous stack.

Nodes launched:
  1. zed_wrapper/zed_node          — ZED camera + VIO (with IMU FUSION)
  2. vision_perception/line_mapper — blue-tape detection
  3. vision_perception/lane_slam   — SLAM v6.0 (pursuit target + legacy guidance)
  4. vision_perception/slam_visualizer — BEV diagnostics (optional)
  5. control_planning/kart_controller  — PD or pure-pursuit (configurable)
  6. vesc_ackermann/ackermann_to_vesc  — Ackermann → VESC
  7. vesc_driver/vesc_driver_node      — serial to VESC

Usage:
  ros2 launch f1tenth_bringup f1tenth_stack.launch.py
Optional args:
  speed_nominal:=0.3            set cruise speed
  controller_mode:=pd           use legacy PD instead of pure-pursuit
  enable_visualizer:=false      skip the BEV visualizer
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
    zed_override = os.path.join(bringup_dir, 'config', 'zed_overrides.yaml')

    speed_arg = DeclareLaunchArgument(
        'speed_nominal', default_value='0.0',
        description='Cruise speed in m/s (starts stopped by default).')
    mode_arg = DeclareLaunchArgument(
        'controller_mode', default_value='pure_pursuit',
        description='"pure_pursuit" or "pd".')
    viz_arg = DeclareLaunchArgument(
        'enable_visualizer', default_value='true',
        description='Enable SLAM BEV visualizer.')

    # -- 1. ZED camera (with IMU FUSION enabled via override) ----------------
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            # The zed_wrapper launch file supports a param-override path.
            # If your zed_wrapper version uses a different keyword, see the
            # manual yaml-edit fallback in the deployment notes.
            'ros_params_override_path': zed_override,
        }.items(),
    )

    # -- 2. Perception -------------------------------------------------------
    perception_node = Node(
        package='vision_perception',
        executable='line_mapper',
        name='vision_perception_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 3. SLAM -------------------------------------------------------------
    slam_node = Node(
        package='vision_perception',
        executable='lane_slam',
        name='lane_slam_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 4. Visualizer (optional) -------------------------------------------
    viz_node = Node(
        package='vision_perception',
        executable='slam_visualizer',
        name='slam_visualizer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visualizer')),
    )

    # -- 5. Controller -------------------------------------------------------
    control_node = Node(
        package='control_planning',
        executable='kart_controller',
        name='control_planning_node',
        output='screen',
        parameters=[
            vesc_params,
            {'controller_mode': LaunchConfiguration('controller_mode'),
             'speed_nominal':   LaunchConfiguration('speed_nominal')},
        ],
    )

    # -- 6. Ackermann → VESC -------------------------------------------------
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 7. VESC driver ------------------------------------------------------
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[vesc_params],
    )

    return LaunchDescription([
        speed_arg, mode_arg, viz_arg,
        zed_node,
        perception_node,
        slam_node,
        viz_node,
        control_node,
        ackermann_to_vesc_node,
        vesc_driver_node,
    ])

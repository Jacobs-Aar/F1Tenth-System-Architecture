"""
f1tenth_bringup/launch/f1tenth_stack.launch.py
================================================
Single entry point for the complete F1Tenth autonomous stack.

Run with:
  ros2 launch f1tenth_bringup f1tenth_stack.launch.py

Optional overrides at launch time:
  ros2 launch f1tenth_bringup f1tenth_stack.launch.py speed_nominal:=0.5

Node graph started by this file:
  zed_wrapper/zed_node          -- ZED 2i camera driver (image + point cloud)
      |  /zed/zed_node/rgb/image_rect_color
      |  /zed/zed_node/point_cloud/cloud_registered
  vision_perception/line_mapper -- blue-tape lane detection
      |  /perception/lane_guidance  (Vector3: lat_err, hdg_err, quality)
  control_planning/kart_controller -- steering + speed commands
      |  /drive  (AckermannDriveStamped)
  vesc_ackermann/ackermann_to_vesc  -- translates drive -> ERPM + servo
      |  /commands/motor/speed  /commands/servo/position
  vesc_driver/vesc_driver_node  -- serial comms to VESC hardware
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -- Shared paths ---------------------------------------------------------
    bringup_dir = get_package_share_directory('f1tenth_bringup')
    vesc_params = os.path.join(bringup_dir, 'config', 'vesc_params.yaml')

    # -- Launch arguments (overridable from command line) ----------------------
    speed_arg = DeclareLaunchArgument(
        'speed_nominal', default_value='1.0',
        description='Cruise speed in m/s')

    # -- 1. ZED 2i Camera Node ------------------------------------------------
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

    # -- 2. Vision Perception Node --------------------------------------------
    perception_node = Node(
        package='vision_perception',
        executable='line_mapper',
        name='vision_perception_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 3. Control Planning Node ---------------------------------------------
    control_node = Node(
        package='control_planning',
        executable='kart_controller',
        name='control_planning_node',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 4. VESC Ackermann Translator -----------------------------------------
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        output='screen',
        parameters=[vesc_params],
    )

    # -- 5. VESC Driver Node --------------------------------------------------
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[vesc_params],
    )

    return LaunchDescription([
        speed_arg,
        zed_node,
        perception_node,
        control_node,
        ackermann_to_vesc_node,
        vesc_driver_node,
    ])

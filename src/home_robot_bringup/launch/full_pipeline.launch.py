#!/usr/bin/env python3
"""
Master launch file for the Cloud-Based Autonomous Home Service Robot.

Launches:
  1. TIAGo in Gazebo with custom home world
  2. Nav2 + SLAM (builds map online, no pre-existing map needed)
  3. MoveIt2 arm planning
  4. All ros2_control controllers (via tiago_bringup)
  5. Task Coordinator node

Usage:
    ros2 launch home_robot_bringup full_pipeline.launch.py

Author: Febin TJ
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==============================================
    #           PACKAGE DIRECTORIES
    # ==============================================
    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')

    # ==============================================
    #        1. TIAGO + GAZEBO + NAV2 + MOVEIT2
    # ==============================================
    # tiago_gazebo.launch.py handles everything:
    #   - Gazebo world loading
    #   - Robot spawn + ros2_control
    #   - tiago_bringup (spawns ALL controllers: mobile_base, arm, torso, head, gripper, jsb)
    #   - Navigation (Nav2 + SLAM when slam:=True)
    #   - MoveIt2
    # world_name is resolved by pal_gazebo_worlds from its worlds/ dir
    tiago_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': 'home_service',
            'navigation': 'True',
            'moveit': 'True',
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
        }.items()
    )

    # ==============================================
    #    NOTE: Controllers are NOT spawned here.
    #    tiago_bringup (via default_controllers.launch.py)
    #    already spawns: mobile_base_controller,
    #    joint_state_broadcaster, arm_controller,
    #    torso_controller, head_controller, gripper_controller.
    # ==============================================

    # ==============================================
    #        2. VISION & COORDINATION (delayed start)
    # ==============================================
    target_color = LaunchConfiguration('target_color')

    food_detector = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='🍎 Starting Food Detector Node...'),
            Node(
                package='task_coordinator',
                executable='food_detector',
                name='food_detector',
                parameters=[{
                    'use_sim_time': True,
                    'target_color': target_color
                }],
            ),
        ]
    )

    task_coordinator = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='🚀 Starting Task Coordinator Node...'),
            Node(
                package='task_coordinator',
                executable='task_coordinator_node',
                name='task_coordinator',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'target_color': target_color
                }],
            ),
        ]
    )

    # ==============================================
    #           LAUNCH DESCRIPTION
    # ==============================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_color',
            default_value='red',
            description='Target color cube to pick (red, green, blue)'),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Whether to run SLAM (online mapping) or AMCL (static map localization)'),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to the static map .yaml file (required if slam:=False)'),

        # Launch the robot simulation, nav, and moveit
        tiago_launch,

        # Launch vision and coordination with delay
        food_detector,
        task_coordinator,

        LogInfo(msg=''),
        LogInfo(msg='  After launch (~45s), trigger the food delivery task:'),
        LogInfo(msg='  ros2 service call /bring_food std_srvs/srv/Trigger "{}"'),
        LogInfo(msg='  OR: ros2 run task_coordinator bring_food'),
        LogInfo(msg=''),
        LogInfo(msg='  To pick a specific color (green/blue):'),
        LogInfo(msg='  ros2 launch home_robot_bringup full_pipeline.launch.py target_color:=green'),
        LogInfo(msg=''),
        LogInfo(msg='  To launch with a static map (no SLAM):'),
        LogInfo(msg='  ros2 launch home_robot_bringup full_pipeline.launch.py slam:=False map:=/home/febintj007/my_map.yaml'),
        LogInfo(msg=''),
    ])

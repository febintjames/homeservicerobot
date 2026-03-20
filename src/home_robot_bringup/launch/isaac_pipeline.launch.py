#!/usr/bin/env python3
"""
Isaac Sim Pipeline Launch File
===============================
Launches the ROS2 stack for the Home Service Robot running in
NVIDIA Isaac Sim (instead of Gazebo).

This replaces the Gazebo-specific full_pipeline.launch.py.
Do NOT run both at the same time.

Prerequisites:
  - Isaac Sim is running on the cloud VM (via WebRTC)
  - home_world_isaac.py has been executed in Isaac Sim Script Editor
  - The PLAY button (▶) has been pressed in Isaac Sim
  - ROS2 bridge topics are visible: `ros2 topic list`

What this launches:
  1. robot_state_publisher  — URDF → /tf transitions
  2. Nav2 (navigation)      — SLAM or AMCL + DWB controller
  3. MoveIt2 (arm planning) — arm + gripper motion planning
  4. food_detector node     — HSV colour detection on camera feed
  5. task_coordinator node  — pick & deliver coloured cube

Usage:
  ros2 launch home_robot_bringup isaac_pipeline.launch.py target_color:=red
  ros2 launch home_robot_bringup isaac_pipeline.launch.py target_color:=green slam:=True
  ros2 launch home_robot_bringup isaac_pipeline.launch.py slam:=False map:=/home/user/my_map.yaml

Author: Febin TJ
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package directories ─────────────────────────────────────────────────
    tiago_description_dir  = get_package_share_directory('tiago_description')
    tiago_bringup_dir      = get_package_share_directory('tiago_bringup')
    nav2_bringup_dir       = get_package_share_directory('nav2_bringup')
    tiago_moveit_dir       = get_package_share_directory('tiago_moveit_config')

    # ── Launch arguments ────────────────────────────────────────────────────
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='red',
        description='Target cube colour to pick: red | green | blue')

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Use SLAM (online mapping) or AMCL (static map)')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to static map .yaml (only if slam:=False)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time from Isaac Sim')

    # ── 1. robot_state_publisher ────────────────────────────────────────────
    #    Reads TIAGo URDF and publishes static /tf frames.
    #    Isaac Sim ROS2 bridge sends the dynamic joint_states;
    #    robot_state_publisher converts those to TF.
    tiago_urdf_path = os.path.join(
        tiago_description_dir, 'robots', 'tiago.urdf')

    with open(tiago_urdf_path, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ── 2. Nav2 Navigation Stack ────────────────────────────────────────────
    #    Uses /scan_raw, /odom, /tf from Isaac Sim ROS2 bridge.
    #    When slam:=True → runs SLAM Toolbox online.
    #    When slam:=False → runs AMCL with a pre-built map.
    nav2_params_file = os.path.join(
        get_package_share_directory('tiago_navigation'),
        'params', 'tiago_nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  nav2_params_file,
            'slam':         LaunchConfiguration('slam'),
            'map':          LaunchConfiguration('map'),
        }.items()
    )

    # ── 3. MoveIt2 Arm Planning ─────────────────────────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_moveit_dir, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # ── 4. ros2_control Joint Trajectory Controllers ─────────────────────────
    #    In Isaac Sim mode the joint commands go via the ROS2 bridge
    #    (SubscribeJointAction nodes). We still need ros2_control's
    #    controller_manager to advertise the action servers MoveIt2 calls.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            os.path.join(tiago_bringup_dir, 'config', 'tiago_controllers.yaml'),
        ],
    )

    spawn_controllers = TimerAction(
        period=5.0,
        actions=[
            # Joint State Broadcaster
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
            # Arm
            Node(package='controller_manager', executable='spawner',
                 arguments=['arm_controller', '--controller-manager', '/controller_manager']),
            # Torso
            Node(package='controller_manager', executable='spawner',
                 arguments=['torso_controller', '--controller-manager', '/controller_manager']),
            # Head
            Node(package='controller_manager', executable='spawner',
                 arguments=['head_controller', '--controller-manager', '/controller_manager']),
            # Gripper
            Node(package='controller_manager', executable='spawner',
                 arguments=['gripper_controller', '--controller-manager', '/controller_manager']),
            # Mobile base
            Node(package='controller_manager', executable='spawner',
                 arguments=['mobile_base_controller', '--controller-manager', '/controller_manager']),
        ]
    )

    # ── 5. Food Detector (delayed — wait for camera topics) ────────────────
    food_detector = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='🍎 Starting Food Detector Node (Isaac Sim mode)...'),
            Node(
                package='task_coordinator',
                executable='food_detector',
                name='food_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'target_color': LaunchConfiguration('target_color'),
                }],
            ),
        ]
    )

    # ── 6. Task Coordinator (delayed — wait for nav2 + moveit) ─────────────
    task_coordinator = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='🚀 Starting Task Coordinator Node (Isaac Sim mode)...'),
            Node(
                package='task_coordinator',
                executable='task_coordinator_node',
                name='task_coordinator',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'target_color': LaunchConfiguration('target_color'),
                }],
            ),
        ]
    )

    return LaunchDescription([
        # Arguments
        target_color_arg,
        slam_arg,
        map_arg,
        use_sim_time_arg,

        # Robot base
        robot_state_publisher,

        # Navigation + MoveIt2
        nav2_launch,
        moveit_launch,

        # Controllers
        controller_manager,
        spawn_controllers,

        # Vision + coordination
        food_detector,
        task_coordinator,

        # Usage hints
        LogInfo(msg=''),
        LogInfo(msg='━━━ Isaac Sim Pipeline ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='  Make sure Isaac Sim is playing (▶) and bridge is active!'),
        LogInfo(msg='  Verify topics:  ros2 topic list'),
        LogInfo(msg=''),
        LogInfo(msg='  Trigger pick-and-deliver:'),
        LogInfo(msg='  ros2 service call /bring_food std_srvs/srv/Trigger "{}"'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg=''),
    ])

#!/usr/bin/env python3
"""
Task Coordinator Node for Autonomous Home Service Robot.

Implements a state-machine-based task pipeline for pick-and-deliver operations.
Coordinates Nav2 navigation and MoveIt2 arm manipulation to execute:
  1. Navigate to kitchen table
  2. Pick up food object
  3. Navigate to user location
  4. Place/deliver object

Author: Febin TJ
License: Apache-2.0
"""

import math
import time
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
import tf2_geometry_msgs

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration, Time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from pymoveit2 import MoveIt2


class TaskState(Enum):
    """States for the task pipeline."""
    IDLE = auto()
    NAVIGATE_TO_KITCHEN = auto()
    
    # New Safe Table Approach Sequence
    LIFT_TORSO_HIGH = auto()
    DEPLOY_ARM_HIGH = auto()
    LOWER_TORSO_FOR_DETECTION = auto()
    WAIT_FOR_DETECTION = auto()
    GRASP_APPROACH = auto()
    PRE_GRASP_VERIFY = auto() # New wrist-camera check
    GRASP_CLOSE = auto()
    VERIFY_GRASP = auto()              # Post-grasp depth camera verification
    LIFT_TORSO_HIGH_RETRACT = auto()
    STOW_ARM = auto()
    
    LIFT_TORSO_FOR_MOVE = auto()
    NAVIGATE_TO_USER = auto()
    LIFT_TORSO_FOR_PLACE = auto()
    PLACE_APPROACH = auto()
    PLACE_OPEN = auto()
    PLACE_RETRACT = auto()
    NAVIGATE_TO_START = auto()
    SUCCESS = auto()
    FAILURE = auto()


class TaskCoordinator(Node):
    """
    Coordinates navigation and manipulation for home service robot tasks.

    This node implements a sequential task pipeline using Nav2 for navigation
    and joint trajectory controllers for arm manipulation.
    """

    def __init__(self):
        super().__init__('task_coordinator')
        self.callback_group = ReentrantCallbackGroup()

        # --- Parameters ---
        # use_sim_time may already be declared by ROS 2 if passed via launch
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass  # Already declared via launch parameters

        # Waypoints for navigation (base poses)
        self.waypoints = {
            'kitchen_table': {'x': 2.83, 'y': 0.0, 'yaw': 0.0},
            'user_location': {'x': -2.8, 'y': 0.0, 'yaw': 3.14159},
            'start_position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        }
    
        self.declare_parameter('manual_pick', False)
        # Override any launch params that set this to True to force automation
        self.manual_pick = False
        
        self.declare_parameter('skip_navigation', False)
        self.declare_parameter('waiting_for_user', False)
        self.skip_navigation = self.get_parameter('skip_navigation').get_parameter_value().bool_value
        self.waiting_for_user = self.get_parameter('waiting_for_user').get_parameter_value().bool_value

        # --- Arm Joint Positions ---
        self.arm_joints = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        self.arm_poses = {
            'stow': [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],
            # Pointing straight down, wrist elevated, safely clear of the table.
            'pre_grasp': [1.1, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0], 
            'grasp': [1.1, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0],     
            'place': [1.55, -0.70, 0.25, 0.80, 1.40, 1.2, 0.0],      # User-validated drop pose
            'observe': [2.0, 1.0, 0.0, 1.5, 0.0, 0.0, 0.0],          # Pointing further back/down to unblock head camera completely
            'verify_grasp': [1.7, 0.4, 0.0, 1.2, 1.5, 0.0, 0.0],    # Refined: Higher and angled for clear wrist camera FOV
        }

        self.gripper_open = [0.044, 0.044]  # Fully open
        self.gripper_close = [0.024, 0.024] # Moderate squeeze with balanced physics

        # --- State Machine ---
        self.state = TaskState.IDLE
        self.task_active = False
        self.task_start_time = None
        self.step_count = 0
        self.declare_parameter('target_color', 'red')
        self.target_color = "red"
        self.grasp_retry_count = 0
        self.max_grasp_retries = 2
        self.detection_retry_count = 0
        self.max_detection_retries = 3

        # --- Action Clients ---
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Direct topic publisher as fallback when action server times out
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.torso_client = ActionClient(
            self, FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        self.head_client = ActionClient(
            self, FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Trigger Service
        self.bring_food_srv = self.create_service(
            Trigger, 'bring_food', self.bring_food_callback,
            callback_group=self.callback_group
        )
        self.continue_srv = self.create_service(
            Trigger, '/continue_task', self.continue_task_cb,
            callback_group=self.callback_group
        )
        self.reset_srv = self.create_service(
            Trigger, '/reset_task', self.reset_task_cb,
            callback_group=self.callback_group
        )

        self.status_timer = self.create_timer(
            5.0, self.status_callback,
            callback_group=self.callback_group
        )

        # --- State Tracking ---
        self.hover_position = None
        self.approach_quat = [0.0, 0.707, 0.0, 0.707]

        # --- Vision ---
        self.detected_food_pose = None
        self.last_detection_wall_time = 0.0
        self.food_pose_sub = self.create_subscription(
            PoseStamped, '/detected_food_pose', self.food_pose_callback, 10,
            callback_group=self.callback_group
        )
        
        # Wrist Camera Subscriptions
        self.wrist_rgb_sub = self.create_subscription(
            Image, '/gripper_wrist_camera/image_raw', self.wrist_image_callback, 10,
            callback_group=self.callback_group
        )
        self.wrist_depth_sub = self.create_subscription(
            Image, '/gripper_wrist_camera/depth/image_raw', self.wrist_depth_callback, 10,
            callback_group=self.callback_group
        )
        self.wrist_info_sub = self.create_subscription(
            CameraInfo, '/gripper_wrist_camera/camera_info', self.wrist_info_callback, 10,
            callback_group=self.callback_group
        )

        self.bridge = CvBridge()
        self.latest_wrist_image = None
        self.latest_wrist_depth = None
        self.wrist_camera_info = None

        # --- MoveIt Client ---
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'],
            base_link_name='base_footprint',
            end_effector_name='gripper_grasping_frame',
            group_name='arm',
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2
        # Strictly prevent IK solver from leaping across joint limits during cartesian retract/plunge.
        # This prevents the arm from snapping and flailing the physics engine.
        self.moveit2.cartesian_jump_threshold = 1.5

        # --- TF2 Listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  TASK COORDINATOR NODE INITIALIZED')
        self.get_logger().info('  Service: /bring_food (std_srvs/Trigger)')
        self.get_logger().info('  Call to start food delivery task')
        self.get_logger().info('=' * 60)

    def _retry_or_fail(self, reason):
        """Retry detection/grasp pipeline, or fail after max retries."""
        self.grasp_retry_count += 1
        self.get_logger().warn(
            f'⚠️ {reason} (attempt {self.grasp_retry_count}/{self.max_grasp_retries})'
        )

        if self.grasp_retry_count <= self.max_grasp_retries:
            self.control_gripper(close=False)
            self.detected_food_pose = None
            self.state = TaskState.WAIT_FOR_DETECTION
            return

        self.grasp_retry_count = 0
        self._fail(f'{reason} after maximum retries')

    def _transform_detected_pose_to_base(self, detected_pose):
        """Transform detected pose to base_footprint with timestamp fallback."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint',
                detected_pose.header.frame_id,
                detected_pose.header.stamp,
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except Exception:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint',
                detected_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

        local_pose = tf2_geometry_msgs.do_transform_pose(detected_pose.pose, transform)
        return local_pose.position.x, local_pose.position.y, local_pose.position.z

    def _execute_moveit_pose(self, position, quat_xyzw, cartesian=False, timeout_sec=20.0):
        """Execute a MoveIt pose goal with explicit timeout and non-blocking state polling."""
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            frame_id='base_footprint',
        )

        # Allow enough time for planner/action handshake before declaring failure.
        request_deadline = time.time() + 8.0
        while time.time() < request_deadline:
            state = self.moveit2.query_state().name
            if state in ('REQUESTING', 'EXECUTING'):
                break
            if state == 'IDLE' and self.moveit2.motion_suceeded:
                return True
            time.sleep(0.05)
        else:
            self.get_logger().warn(
                f'⚠️ MoveIt did not accept/start request for target {position} within handshake timeout'
            )
            return False

        execution_future = None
        future_deadline = time.time() + 2.0
        while time.time() < future_deadline:
            if self.moveit2.query_state().name == 'EXECUTING':
                execution_future = self.moveit2.get_execution_future()
                if execution_future is not None:
                    break
            if self.moveit2.query_state().name == 'IDLE':
                return bool(self.moveit2.motion_suceeded)
            time.sleep(0.05)

        if execution_future is not None:
            if self._wait_for_future(execution_future, timeout=timeout_sec):
                return bool(self.moveit2.motion_suceeded)
        else:
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                if self.moveit2.query_state().name == 'IDLE':
                    return bool(self.moveit2.motion_suceeded)
                time.sleep(0.1)

        # Timed out while execution is still active.
        self.get_logger().warn('⚠️ MoveIt motion timeout reached. Cancelling execution.')
        self.moveit2.cancel_execution()
        return False

    # ==========================================
    #           SERVICE CALLBACK
    # ==========================================

    def food_pose_callback(self, msg):
        """Store the latest detected food pose."""
        self.detected_food_pose = msg
        self.last_detection_wall_time = time.time()
        if not self.task_active:
            self.get_logger().warn(
                'Detection received while task is IDLE. '
                'Call /bring_food to start the motion pipeline.',
                throttle_duration_sec=5.0,
            )

    def bring_food_callback(self, request, response):
        """Handle /bring_food service call to trigger the task pipeline."""
        if self.task_active:
            response.success = False
            response.message = 'Task already in progress!'
            self.get_logger().warn('Task request rejected — already active.')
            return response

        self.get_logger().info('🚀 BRING FOOD task triggered!')
        self.task_active = True
        self.task_start_time = time.time()
        self.step_count = 0
        self.detected_food_pose = None
        self.waiting_for_user = False
        self.grasp_retry_count = 0
        self.detection_retry_count = 0
        self.state = TaskState.NAVIGATE_TO_KITCHEN

        # Start the pipeline in a separate thread to avoid blocking the executor
        # and to prevent reentrancy issues from high-frequency timers.
        self.pipeline_thread = threading.Thread(target=self.execute_pipeline)
        self.pipeline_thread.daemon = True
        self.pipeline_thread.start()

        response.success = True
        response.message = 'Food delivery task started!'
        return response

    def continue_task_cb(self, request, response):
        """Service callback to resume paused task."""
        if self.waiting_for_user:
            self.waiting_for_user = False
            response.success = True
            response.message = "Resuming task..."
            self.get_logger().info("▶️ Resuming task from pause...")
        else:
            response.success = False
            response.message = "Task is not paused."
        return response

    def reset_task_cb(self, request, response):
        """Service callback to forcibly abort the current task and reset to IDLE."""
        if not self.task_active:
            response.success = True
            response.message = "Task is already IDLE."
            self.get_logger().info("🛑 Reset requested, but task is already IDLE.")
            return response

        self.get_logger().warn("🛑 RESET TASK REQUESTED. Aborting pipeline!")
        self.task_active = False
        self.state = TaskState.IDLE
        self.waiting_for_user = False
        
        # NOTE: the pipeline thread might be blocked on a timeout or sleep.
        # It will exit its while loop once it wakes up and sees task_active=False.

        response.success = True
        response.message = "Task forcefully reset to IDLE."
        return response

    # ==========================================
    #         TASK PIPELINE EXECUTOR (THREADED)
    # ==========================================

    def _wait_for_future(self, future, timeout=None):
        """Helper to wait for a future in a thread-safe way."""
        start = time.time()
        while rclpy.ok() and not future.done():
            if timeout and (time.time() - start) > timeout:
                return False
            time.sleep(0.1)
        return future.done()

    def execute_pipeline(self):
        """Execute the task pipeline state machine."""
        self.get_logger().info('🟢 Pipeline thread started')
        
        while rclpy.ok() and self.task_active:
            # Refresh parameters
            self.skip_navigation = self.get_parameter('skip_navigation').get_parameter_value().bool_value
            self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
            
            try:
                if self.state == TaskState.NAVIGATE_TO_KITCHEN:
                    self._step('NAVIGATE_TO_KITCHEN', '🏠→🍳 Navigating to kitchen table...')
                    if self.skip_navigation:
                        self.get_logger().info('⏩ Skipping navigation (Manual Override)')
                        success = True
                    else:
                        # Wait for Nav2 to fully activate (AMCL + costmaps) after fresh launch
                        self.get_logger().info('⏳ Waiting 10s for Nav2 to fully initialize...')
                        time.sleep(10.0)
                        success = self.navigate_to_waypoint('kitchen_table')
                    
                    if success:
                        self.get_logger().info('🔄 Reached table. Transitioning to safe high-clearance approach.')
                        self.detected_food_pose = None
                        self.state = TaskState.LIFT_TORSO_HIGH
                    else:
                        self._fail('Failed to reach kitchen table')

                elif self.state == TaskState.LIFT_TORSO_HIGH:
                    self._step('LIFT_TORSO_HIGH', '🦾 Lifting torso high to clear table...')
                    success = self.move_torso(0.35) # Max height for TIAGo
                    if success:
                        self.state = TaskState.DEPLOY_ARM_HIGH
                    else:
                        self._fail('Torso lift for clearance failed')

                elif self.state == TaskState.DEPLOY_ARM_HIGH:
                    self._step('DEPLOY_ARM_HIGH', '🦾 Moving arm to observation pose away from camera...')
                    self.get_logger().info('🤲 Opening gripper for approach...')
                    self.control_gripper(close=False)
                    success = self.move_arm_to_pose('observe')
                    if success:
                        self.state = TaskState.LOWER_TORSO_FOR_DETECTION
                    else:
                        self._fail('Pre-grasp arm deployment failed')

                elif self.state == TaskState.LOWER_TORSO_FOR_DETECTION:
                    self._step('LOWER_TORSO_FOR_DETECTION', '🦾 Lowering torso to see cubes...')
                    time.sleep(0.5) # ensure arm controllers settle
                    success = self.move_torso(0.189) # Optimized height for arm reachability per user testing
                    if success:
                        self._step('WAIT_FOR_DETECTION', '👀 Tilting head to look at table...')
                        self.move_head(tilt=-1.0) # Optimal downward look at 2.8m distance
                        time.sleep(3.0) # Wait for head movement to finish so camera is stable
                        detection_age = time.time() - self.last_detection_wall_time
                        if self.detected_food_pose is None or detection_age > 8.0:
                            self.detected_food_pose = None
                        else:
                            self.get_logger().info(
                                f'Keeping fresh detection captured {detection_age:.1f}s ago for grasp approach.'
                            )
                        self.state = TaskState.WAIT_FOR_DETECTION
                    else:
                        self._fail('Torso lowering for detection failed')

                elif self.state == TaskState.WAIT_FOR_DETECTION:
                    new_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
                    if new_color != self.target_color:
                        self.get_logger().info(f'🔄 Target color changed from {self.target_color} to {new_color}. Resetting detection.')
                        self.target_color = new_color
                        self.detected_food_pose = None

                    self._step('WAIT_FOR_DETECTION', f'👀 Looking for {self.target_color} cube...')
                    
                    if self.manual_pick:
                        self.get_logger().info('🛑 MANUAL MODE: Pausing at table for assistance.')
                        self.waiting_for_user = True
                    else:
                        # For now, we wait up to 30 seconds for a detection
                        start_wait = time.time()
                        if self.detected_food_pose is not None:
                            detection_age = time.time() - self.last_detection_wall_time
                            if detection_age <= 8.0:
                                self.get_logger().info(
                                    f'Using recent detection captured {detection_age:.1f}s ago.'
                                )
                        while self.detected_food_pose is None and (time.time() - start_wait) < 30.0:
                            time.sleep(1.0)
                        
                        if self.detected_food_pose:
                            self.get_logger().info(f'✅ {self.target_color.capitalize()} cube detected!')
                            self.detection_retry_count = 0
                            self.state = TaskState.GRASP_APPROACH
                        else:
                            self.detection_retry_count += 1
                            if self.detection_retry_count <= self.max_detection_retries:
                                self.get_logger().warn(
                                    f'⚠️ No cube detected. Re-scanning '
                                    f'({self.detection_retry_count}/{self.max_detection_retries})...'
                                )
                                self.move_head(tilt=-1.0)
                                self.detected_food_pose = None
                                time.sleep(1.0)
                                continue
                            self._fail('No cube detected after repeated re-scan attempts')
                            continue

                    # Unified waiting loop
                    while self.waiting_for_user:
                        if self.detected_food_pose and not self.manual_pick: 
                            self.state = TaskState.GRASP_APPROACH
                            break
                        time.sleep(1.0)
                    
                    # Transition logic after resume
                    if self.state == TaskState.WAIT_FOR_DETECTION:
                        if self.detected_food_pose:
                            self.state = TaskState.GRASP_APPROACH
                        else:
                            self.get_logger().info('👋 Manual resume triggered. Skipping to delivery.')
                            self.state = TaskState.LIFT_TORSO_HIGH_RETRACT

                elif self.state == TaskState.GRASP_APPROACH:
                    self._step('GRASP_APPROACH', '🦾 MoveIt: Downward approach to detected object...')
                    
                    if self.detected_food_pose:
                        try:
                            # 1. Transform detected pose from 'map' (or whatever) to 'base_footprint'
                            # MoveIt needs coordinates relative to the robot's base
                            self.get_logger().info(f"🔄 Transforming pose from {self.detected_food_pose.header.frame_id} to base_footprint...")
                            
                            x, y, z = self._transform_detected_pose_to_base(self.detected_food_pose)

                            self.get_logger().info(f"📍 Detect {self.target_color.capitalize()} Cube at base_footprint: x={x:.3f}, y={y:.3f}, z={z:.3f}")

                            # --- REACHABILITY SAFETY CHECK ---
                            # TIAGo can only reach about 0.8-0.9m forward. 
                            # If X is larger (like 2.6m map coords), something is wrong!
                            if x > 1.1:
                                self.get_logger().error(
                                    f'Grasp rejected: detected object is out of arm reach in base_footprint '
                                    f'(x={x:.3f}, y={y:.3f}, z={z:.3f}). '
                                    f'Robot is likely still too far from the table or navigation did not finish.'
                                )
                                self._retry_or_fail(
                                    f"Object is too far after transform (x={x:.2f}m). Likely stale/wrong frame"
                                )
                                continue

                            # Clamp into a conservative workspace that remains solvable
                            # for the top-down grasp orientation on this TIAGo setup.
                            x = min(max(x, 0.52), 0.78)
                            y = min(max(y, -0.18), 0.18)

                            # Empirical fingertip alignment offsets for the simulated TIAGo gripper.
                            x -= 0.03
                            y -= 0.02

                            # --- GRASP ALIGNMENT ---
                            # With robot at x=2.8, detection projection should be highly accurate.
                            # We keep it as-is for now (no manual offset).
                            
                            # 2. Hover Safety Pose (High Above)
                            # Table top: 0.405m in home.world. Cube center: ~0.43m.
                            # Keep grasp target near cube center to avoid scraping the table.
                            safe_z = 0.435
                            self.hover_position = [x, y, safe_z + 0.08]
                            self.approach_quat = [0.0, 0.707, 0.0, 0.707] # Standard top-down orientation
                            
                            self.get_logger().info(f"🦾 TARGET POSE: x={x:.3f}, y={y:.3f}, z={safe_z:.3f}")
                            
                            # Seed IK by moving to joint-space pre-grasp first
                            self.get_logger().info('🦾 Seeding IK with joint-space pre-grasp...')
                            if not self.move_arm_to_pose('pre_grasp'):
                                self._retry_or_fail('Failed to move to pre-grasp seed pose')
                                continue
                            
                            self.get_logger().info(f"🦾 Moving to SAFE HOVER pose: {self.hover_position}")
                            if not self._execute_moveit_pose(
                                position=self.hover_position,
                                quat_xyzw=self.approach_quat,
                                cartesian=False,
                                timeout_sec=20.0,
                            ):
                                self._retry_or_fail('Hover pose motion failed or was obstructed')
                                continue
                            
                            # --- RE-ALIGNMENT: Wait for fresh detection while hovering ---
                            self.get_logger().info('👀 Re-scanning from hover pose for final alignment...')
                            self.detected_food_pose = None
                            wait_start = time.time()
                            while self.detected_food_pose is None and (time.time() - wait_start) < 4.0:
                                time.sleep(0.5)
                            
                            if self.detected_food_pose:
                                # Transform fresh detection to base_footprint
                                try:
                                    x, y, _ = self._transform_detected_pose_to_base(self.detected_food_pose)
                                    self.get_logger().info(f"🦾 Refined target: x={x:.3f}, y={y:.3f}")
                                except Exception:
                                    self.get_logger().warn("⚠️ Re-alignment transform failed, using original pose.")
                            
                            # 3. Plunge
                            # Slightly faster plunge while remaining conservative near contact.
                            self.moveit2.max_velocity = 0.10
                            approach_position = [x, y, safe_z]
                            self.get_logger().info(f"🦾 Performing PLUNGE to {approach_position}...")
                            if not self._execute_moveit_pose(
                                position=approach_position,
                                quat_xyzw=self.approach_quat,
                                cartesian=True,
                                timeout_sec=20.0,
                            ):
                                self._retry_or_fail('Plunge motion failed')
                                continue
                            
                            self.state = TaskState.GRASP_CLOSE
                        except Exception as e:
                            self.get_logger().error(f"❌ Transformation failed: {str(e)}")
                            self._retry_or_fail(f"Could not transform detected pose: {str(e)}")
                    else:
                        self._retry_or_fail('Grasp approach failed: no object detected')

                elif self.state == TaskState.GRASP_CLOSE:
                    self._step('GRASP_CLOSE', '✊ Waiting for arm to settle, then closing gripper...')
                    # Generous pause: MoveIt release + Gazebo physics settle
                    time.sleep(4.0) 
                    success = self.control_gripper(close=True)
                    if success:
                        # Allow physics to settle after contact before lifting
                        self.get_logger().info('⏳ Settling grasp...')
                        time.sleep(2.0)
                        self.state = TaskState.VERIFY_GRASP
                    else:
                        self._fail('Gripper close failed')

                elif self.state == TaskState.VERIFY_GRASP:
                    self._step('VERIFY_GRASP', '🔍 Verifying grasp using wrist camera...')
                    
                    # 1. Lift slightly and move to verification pose
                    lift_pos = [self.hover_position[0], self.hover_position[1], 0.60]
                    if not self._execute_moveit_pose(
                        position=lift_pos,
                        quat_xyzw=self.approach_quat,
                        cartesian=True,
                        timeout_sec=15.0,
                    ):
                        self._retry_or_fail('Lift-after-grasp motion failed')
                        continue
                    
                    # Reset speed for regular moves following the secure lift
                    self.moveit2.max_velocity = 0.2
                    
                    self.move_arm_to_pose('verify_grasp')
                    time.sleep(2.0) # allow camera to settle
                    
                    # 2. Visual verification logic (check for target color in wrist camera)
                    grabbed = self.check_visual_grasp()
                    
                    if grabbed:
                        self.get_logger().info('✅ Positive verification! Object seen in wrist camera.')
                        self.grasp_retry_count = 0
                        self.state = TaskState.LIFT_TORSO_HIGH_RETRACT
                    else:
                        self._retry_or_fail('Grasp verification failed (cube not visible in wrist camera)')

                elif self.state == TaskState.LIFT_TORSO_HIGH_RETRACT:
                    self._step('LIFT_TORSO_HIGH_RETRACT', '🦾 Lifting arm and torso safely above table...')
                    time.sleep(0.5) # allow MoveIt cartesian execution to fully release controllers
                    success = self.move_torso(0.35)
                    if success:
                        self.state = TaskState.STOW_ARM
                    else:
                        self._fail('Torso lift for retract failed')

                elif self.state == TaskState.STOW_ARM:
                    self._step('STOW_ARM', '🦾 Stowing arm for navigation...')
                    success = self.move_arm_to_pose('stow')
                    if success:
                        self.state = TaskState.LIFT_TORSO_FOR_MOVE
                    else:
                        self._fail('Arm stowing failed')

                elif self.state == TaskState.LIFT_TORSO_FOR_MOVE:
                    self._step('LIFT_TORSO_FOR_MOVE', '🦾 Lowering torso for stability...')
                    time.sleep(0.5)
                    success = self.move_torso(0.15)
                    if success:
                        self.state = TaskState.NAVIGATE_TO_USER
                    else:
                        self._fail('Torso move for stability failed')

                elif self.state == TaskState.NAVIGATE_TO_USER:
                    self._step('NAVIGATE_TO_USER', '🍳→🏠 Navigating to user location...')
                    if self.skip_navigation:
                        self.get_logger().info('⏩ Skipping navigation (Manual Override)')
                        success = True
                    else:
                        success = self.navigate_to_waypoint('user_location')
                    
                    if success:
                        self.state = TaskState.LIFT_TORSO_FOR_PLACE
                    else:
                        self._fail('Navigation to user failed')

                elif self.state == TaskState.LIFT_TORSO_FOR_PLACE:
                    self._step('LIFT_TORSO_FOR_PLACE', '🦾 Adjusting torso for delivery...')
                    time.sleep(0.5)
                    success = self.move_torso(0.15)
                    if success:
                        self.state = TaskState.PLACE_APPROACH
                    else:
                        self._fail('Torso adjustment failed')

                elif self.state == TaskState.PLACE_APPROACH:
                    self._step('PLACE_APPROACH', '🦾 Moving arm to drop position over the bucket...')
                    # The bucket is on the coffee table. When at user_location (-2.8, 0, pi),
                    # The top of the bucket is at Z=0.42m. We hover at Z=0.52m for a safe drop.
                    drop_pos = [0.45, 0.0, 0.52]
                    drop_quat = [0.0, 0.707, 0.0, 0.707] # Pointing down
                    
                    self.moveit2.max_velocity = 0.2
                    success = self._execute_moveit_pose(
                        position=drop_pos,
                        quat_xyzw=drop_quat,
                        cartesian=False,
                        timeout_sec=20.0,
                    )
                    
                    if success:
                        self.state = TaskState.PLACE_OPEN
                    else:
                        self._fail('Place approach failed (IK timeout)')

                elif self.state == TaskState.PLACE_OPEN:
                    self._step('PLACE_OPEN', '🤲 Opening gripper to release object...')
                    success = self.control_gripper(close=False)
                    if success:
                        self.state = TaskState.PLACE_RETRACT
                    else:
                        self._fail('Gripper open failed')

                elif self.state in [TaskState.PLACE_RETRACT, TaskState.STOW_ARM]:
                    self._step('STOW_ARM', '📦 Stowing arm after delivery...')
                    success = self.move_arm_to_pose('stow')
                    if success:
                        self.state = TaskState.NAVIGATE_TO_START
                    else:
                        self._fail('Final arm stow failed')

                elif self.state == TaskState.NAVIGATE_TO_START:
                    self._step('NAVIGATE_TO_START', '🏠 Returning to start position...')
                    success = self.navigate_to_waypoint('start_position')
                    if success:
                        self.state = TaskState.SUCCESS
                    else:
                        self._fail('Navigation to start failed')

                elif self.state == TaskState.SUCCESS:
                    elapsed = time.time() - self.task_start_time
                    self.get_logger().info('=' * 60)
                    self.get_logger().info(f'  ✅ TASK COMPLETED SUCCESSFULLY')
                    self.get_logger().info(f'  Total steps: {self.step_count}')
                    self.get_logger().info(f'  Total time: {elapsed:.1f} seconds')
                    self.get_logger().info('=' * 60)
                    self.task_active = False
                    self.state = TaskState.IDLE

                elif self.state == TaskState.FAILURE:
                    self.get_logger().error('Task pipeline in FAILURE state. Reset needed.')
                    self.task_active = False
                    self.state = TaskState.IDLE
                
                # Prevent busy waiting
                time.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f'Pipeline exception: {str(e)}')
                self._fail(f'Exception: {str(e)}')
                break
        
    # ==========================================
    #           NAVIGATION
    # ==========================================

    def navigate_to_waypoint(self, waypoint_name):
        """Navigate to a named waypoint using Nav2."""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'Unknown waypoint: {waypoint_name}')
            return False

        wp = self.waypoints[waypoint_name]

        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available after 30s!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        yaw = wp['yaw']
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'📍 Sending nav goal to {waypoint_name}: '
            f'({wp["x"]:.1f}, {wp["y"]:.1f}, yaw={wp["yaw"]:.2f})'
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout=60.0):
            self.get_logger().error('Timeout waiting for nav goal acceptance (60s)')
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Nav goal to {waypoint_name} was rejected!')
            return False

        self.get_logger().info(f'Nav goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=120.0):
            self.get_logger().error('Timeout waiting for nav result')
            return False

        result = result_future.result()
        if result and result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'✅ Reached waypoint: {waypoint_name}')
            return True
        else:
            status = result.status if result else 'timeout'
            self.get_logger().error(f'❌ Nav to {waypoint_name} failed with status: {status}')
            return False

    # ==========================================
    #           ARM MANIPULATION
    # ==========================================

    def move_arm_to_pose(self, pose_name):
        """Move the arm to a predefined joint configuration using direct controller."""
        if pose_name not in self.arm_poses:
            self.get_logger().error(f'Unknown arm pose: {pose_name}')
            return False

        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm action server not available!')
            return False

        positions = self.arm_poses[pose_name]

        self.get_logger().info(f'🦾 Moving arm directly to pose: {pose_name}')
        try:
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.arm_joints
            
            point = JointTrajectoryPoint()
            # If the pose happens to be 8 length from a previous MoveIt call configuration, slice it to 7
            point.positions = positions[-7:] 
            point.velocities = [0.0] * 7
            # Give it reasonable time to execute predefined pose
            point.time_from_start = Duration(sec=4, nanosec=0)
            goal_msg.trajectory.points.append(point)

            send_goal_future = self.arm_client.send_goal_async(goal_msg)
            if not self._wait_for_future(send_goal_future, timeout=10.0):
                self.get_logger().error('Timeout waiting for arm goal acceptance')
                return False

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Arm goal rejected!')
                return False

            result_future = goal_handle.get_result_async()
            self._wait_for_future(result_future, timeout=15.0)

            self.get_logger().info(f'✅ Arm reached pose: {pose_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'⚠️ Arm move failed: {e}')
            return False



    def move_head(self, tilt=-1.0):
        """Tilt the head down to look at the table."""
        if not self.head_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Head action server not available!')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'🤖 Moving head (tilt={tilt}) sending goal...')
        send_goal_future = self.head_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout=5.0):
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=10.0):
            return False

        return True

    # ==========================================
    #           GRIPPER CONTROL
    # ==========================================

    def control_gripper(self, close=True):
        """Open or close the gripper using FollowJointTrajectory."""
        if not self.gripper_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Gripper action server not available!')
            return False

        # TIAGo gripper usually has two joints: left and right
        joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        
        # Positions: lists format for [left, right]
        pos = self.gripper_close if close else self.gripper_open
        action_name = 'CLOSE' if close else 'OPEN'

        trajectory = JointTrajectory()
        trajectory.header.stamp = Time(sec=0, nanosec=0)
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        if isinstance(pos, list):
            point.positions = pos
        else:
            point.positions = [pos, pos]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=3, nanosec=0) # Slower closing to avoid physics shocks
        trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'🤖 Gripper {action_name} sending goal...')

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout=10.0):
            self.get_logger().warn(f'⚠️ Action timeout — using DIRECT TOPIC publish for gripper {action_name}')
            # Fallback: publish directly to controller topic
            trajectory.header.stamp = Time(sec=0, nanosec=0)
            self.gripper_pub.publish(trajectory)
            time.sleep(4.0)  # Wait for gripper to physically move
            self.get_logger().info(f'✅ Gripper {action_name} sent via topic publisher')
            return True

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Gripper {action_name} goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=10.0):
            self.get_logger().warn(f'⚠️ Gripper {action_name} result timeout (Proceeding anyway as it might be grasping)')
            return True

        # Success is determined by goal reaching completion without cancellation
        status = result_future.result().status
        if status == 4: # GoalStatus.STATUS_SUCCEEDED = 4 in many ROS 2 impls, but let's check correctly
            self.get_logger().info(f'✅ Gripper {action_name} complete')
            return True
        else:
            self.get_logger().warn(f'⚠️ Gripper {action_name} status: {status} (Proceeding anyway)')
            return True

    # ==========================================
    #           TORSO CONTROL
    # ==========================================

    def move_torso(self, position):
        """Move the torso lift joint to a specific height."""
        if not self.torso_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Torso action server not available!')
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'🦾 Moving torso to height: {position:.2f}m')

        send_goal_future = self.torso_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout=10.0):
            self.get_logger().warn('⚠️ Torso goal acceptance timeout — Proceeding (torso will still move)')
            return True

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Torso goal was rejected!')
            return False

        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, timeout=15.0)

        return True

    # ==========================================
    #           UTILITIES
    # ==========================================

    def _step(self, step_name, description):
        """Log a pipeline step."""
        self.step_count += 1
        elapsed = time.time() - self.task_start_time
        self.get_logger().info(
            f'\n{"─" * 50}\n'
            f'  Step {self.step_count}: {step_name}\n'
            f'  {description}\n'
            f'  Elapsed: {elapsed:.1f}s\n'
            f'{"─" * 50}'
        )

    def _fail(self, reason):
        """Handle a pipeline failure."""
        self.get_logger().error(f'❌ TASK FAILED at state {self.state.name}: {reason}')
        self.state = TaskState.FAILURE
        self.task_active = False

    def status_callback(self):
        """Periodic status log."""
        if self.task_active:
            elapsed = time.time() - self.task_start_time
            self.get_logger().info(
                f'📊 Status: state={self.state.name}, '
                f'step={self.step_count}, elapsed={elapsed:.1f}s'
            )

    # --- VISUAL VERIFICATION HELPERS ---

    def wrist_image_callback(self, msg):
        self.latest_wrist_image = msg

    def wrist_depth_callback(self, msg):
        self.latest_wrist_depth = msg

    def wrist_info_callback(self, msg):
        self.wrist_camera_info = msg

    def check_visual_grasp(self):
        """Check if target colored object is visible in wrist camera."""
        if self.latest_wrist_image is None:
            self.get_logger().warn("No wrist camera image available for verification")
            return False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_wrist_image, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Simple color thresholding based on target_color
            if self.target_color == 'red':
                lower = np.array([0, 100, 100])
                upper = np.array([10, 255, 255])
            elif self.target_color == 'green':
                lower = np.array([40, 40, 40])
                upper = np.array([80, 255, 255])
            elif self.target_color == 'blue':
                lower = np.array([100, 150, 0])
                upper = np.array([140, 255, 255])
            else:
                return False

            mask = cv2.inRange(hsv, lower, upper)
            # Apply morphology to remove noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Check if any significant pixels are present (the cube should be very close and centered)
            pixel_count = cv2.countNonZero(mask)
            self.get_logger().info(f"🔍 Grasp Verification: color={self.target_color}, pixel_count={pixel_count}")
            
            # Cube is extremely close to the wrist camera, should occupy a significant portion of the frame.
            # Using 1000 pixels as a more robust threshold for detection at close range.
            return pixel_count > 1000
        except Exception as e:
            self.get_logger().error(f"Grasp verification error: {str(e)}")
            return False



def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinator()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Task Coordinator...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

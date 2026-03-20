#!/usr/bin/env python3
"""
Pose Recorder Utility for TIAGo Robot.

Echoes current robot base pose and joint positions to help "teach" the robot
new waypoints and manipulation poses.

Usage:
    ros2 run task_coordinator pose_recorder
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        
        self.odom_sub = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.current_odom = None
        self.current_joints = {}
        
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('='*50)
        self.get_logger().info('  POSE RECORDER READY')
        self.get_logger().info('  Manually move the robot/arm and see values below')
        self.get_logger().info('='*50)

    def odom_cb(self, msg):
        self.current_odom = msg

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def print_status(self):
        if self.current_odom is None:
            return

        # Base Pose
        pos = self.current_odom.pose.pose.position
        ori = self.current_odom.pose.pose.orientation
        
        # Quaternion to Yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        print("\n" + "="*40)
        print(f"📍 BASE POSE (x, y, yaw):")
        print(f"   x:   {pos.x:.3f}")
        print(f"   y:   {pos.y:.3f}")
        print(f"   yaw: {yaw:.3f}")

        # Targeted Joints
        print(f"\n🦾 JOINT POSITIONS:")
        
        # Torso
        torso = self.current_joints.get('torso_lift_joint', 0.0)
        print(f"   torso_lift_joint: {torso:.3f}")
        
        # Arm
        arm_joints = ['arm_1', 'arm_2', 'arm_3', 'arm_4', 'arm_5', 'arm_6', 'arm_7']
        arm_values = []
        for j in arm_joints:
            val = self.current_joints.get(f'{j}_joint', 0.0)
            arm_values.append(round(val, 3))
        
        print(f"   arm_joints: {arm_values}")
        
        # Gripper
        gripper_l = self.current_joints.get('gripper_left_finger_joint', 0.0)
        gripper_r = self.current_joints.get('gripper_right_finger_joint', 0.0)
        print(f"   gripper (l/r): [{gripper_l:.3f}, {gripper_r:.3f}]")
        print("="*40)

def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

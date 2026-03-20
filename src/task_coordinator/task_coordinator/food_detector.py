#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math

class FoodDetector(Node):
    def __init__(self):
        super().__init__('food_detector')
        
        # Use sim time if available
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass
        
        self.bridge = CvBridge()
        
        # Subscriptions
        self.rgb_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/head_front_camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/camera_info',
            self.info_callback,
            10,
        )
        self.rgb_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.info_callback,
            10,
        )
        
        # Parameter for dynamic color change
        self.declare_parameter('target_color', 'red')
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_food_pose', 10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.depth_image = None
        self.camera_info = None
        
        self.get_logger().info("🍎 Food Detector initialized. Looking for colors (red, green, blue)...")

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            # Depth images in Gazebo are often 32FC1 (meters)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {str(e)}")

    def rgb_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            missing = []
            if self.depth_image is None:
                missing.append('depth_image')
            if self.camera_info is None:
                missing.append('camera_info')
            self.get_logger().warn(
                f"Waiting for detector inputs: {', '.join(missing)}",
                throttle_duration_sec=5.0,
            )
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {str(e)}")
            return

        # Update target color from parameter
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()

        # HSV segmentation based on target color
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        if self.target_color == "red":
            lower1 = np.array([0, 40, 40])
            upper1 = np.array([20, 255, 255])
            lower2 = np.array([155, 40, 40])
            upper2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif self.target_color == "green":
            lower = np.array([35, 40, 40])
            upper = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        elif self.target_color == "blue":
            lower = np.array([100, 40, 40])
            upper = np.array([130, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        else:
            self.get_logger().warn(f"Unknown target color: {self.target_color}", throttle_duration_sec=5.0)
            return
        
        # Clean up mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return

        # Sort contours by area descending, and try to find one that is reasonably sized
        valid_contours = [c for c in contours if 100 < cv2.contourArea(c) < 50000]
        
        if valid_contours:
            largest_contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            self.get_logger().info(f"{self.target_color.capitalize()} object found! Area: {area}", throttle_duration_sec=2.0)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                # Get depth from depth image at pixel (u,v)
                try:
                    z = self.depth_image[v, u]
                    
                    # Handle NaN or zero depth by checking neighbors
                    if np.isnan(z) or z <= 0.1:
                        window = self.depth_image[max(0,v-3):min(v+4, self.depth_image.shape[0]), 
                                                  max(0,u-3):min(u+4, self.depth_image.shape[1])]
                        z = np.nanmedian(window)

                    if not np.isnan(z) and 0.2 < z < 2.0: # Valid range filter
                        self.project_and_publish(
                            u,
                            v,
                            z,
                            msg.header.frame_id,
                            largest_contour,
                        )
                except Exception as e:
                    self.get_logger().warn(f"Depth lookup failed: {str(e)}")

    def project_and_publish(self, u, v, z, camera_frame, contour):
        depth_h, depth_w = self.depth_image.shape[:2]
        rgb_h = int(self.camera_info.height) if self.camera_info is not None else depth_h
        rgb_w = int(self.camera_info.width) if self.camera_info is not None else depth_w

        scale_x = depth_w / float(rgb_w)
        scale_y = depth_h / float(rgb_h)
        scaled_contour = contour.astype(np.float32).copy()
        scaled_contour[:, 0, 0] *= scale_x
        scaled_contour[:, 0, 1] *= scale_y
        scaled_contour = np.round(scaled_contour).astype(np.int32)

        contour_mask = np.zeros((depth_h, depth_w), dtype=np.uint8)
        cv2.drawContours(contour_mask, [scaled_contour], -1, 255, thickness=cv2.FILLED)

        contour_depths = self.depth_image[contour_mask > 0]
        valid_depths = contour_depths[np.isfinite(contour_depths)]
        valid_depths = valid_depths[(valid_depths > 0.15) & (valid_depths < 2.0)]

        if valid_depths.size > 0:
            # Prefer the nearest consistent depth on the object mask, not the table behind it.
            z = float(np.percentile(valid_depths, 10))
            nearest_indices = np.argwhere(
                (contour_mask > 0)
                & np.isfinite(self.depth_image)
                & (np.abs(self.depth_image - z) < 0.05)
            )
            if nearest_indices.size > 0:
                depth_v = int(np.median(nearest_indices[:, 0]))
                depth_u = int(np.median(nearest_indices[:, 1]))
            else:
                depth_u = int(np.clip(round(u * scale_x), 0, depth_w - 1))
                depth_v = int(np.clip(round(v * scale_y), 0, depth_h - 1))
            self.get_logger().info(
                f"Using robust depth estimate near centroid: z={z:.3f} m "
                f"from {valid_depths.size} depth samples",
                throttle_duration_sec=2.0,
            )
        else:
            self.get_logger().warn(
                'No valid depth values near detected object centroid.',
                throttle_duration_sec=2.0,
            )
            return

        # Camera intrinsics (K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1])
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        # Standard projection: 
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # In ROS Optical Frame: Z is forward, X is right, Y is down
        cam_x = float((depth_u - cx) * z / fx)
        cam_y = float((depth_v - cy) * z / fy)
        cam_z = float(z)
        
        # Create PoseStamped in CAMERA OPTICAL frame
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = camera_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = cam_x
        pose_msg.pose.position.y = cam_y
        pose_msg.pose.position.z = cam_z
        pose_msg.pose.orientation.w = 1.0
        
        try:
            # Look up transform from camera to robot base (KINEMATIC ONLY, NO MAP DRIFT)
            now = rclpy.time.Time()
            target_frame = 'base_footprint'
            transform = self.tf_buffer.lookup_transform(target_frame, camera_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Transform the pose
            pose_base = tf2_geometry_msgs.do_transform_pose(pose_msg.pose, transform)
            
            final_pose = PoseStamped()
            final_pose.header.frame_id = target_frame
            final_pose.header.stamp = pose_msg.header.stamp
            final_pose.pose = pose_base
            
            # --- SAFETY FILTER ---
            if final_pose.pose.position.z < 0.15:
                return

            self.pose_pub.publish(final_pose)
            self.get_logger().info(f"📍 Detect {self.target_color.capitalize()} Cube at {target_frame}: x={final_pose.pose.position.x:.3f}, y={final_pose.pose.position.y:.3f}, z={final_pose.pose.position.z:.3f}", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().warn(f"Transform to {target_frame} failure: {str(e)}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = FoodDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()

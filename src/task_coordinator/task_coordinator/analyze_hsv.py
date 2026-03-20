#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorAnalyzer(Node):
    def __init__(self):
        super().__init__('color_analyzer')
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_sub = self.create_subscription(
            Image, 
            '/head_front_camera/depth/image_raw',
            self.depth_callback, 
            10)
        self.sub = self.create_subscription(
            Image, 
            '/head_front_camera/rgb/image_raw',
            self.callback, 
            10)
            
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            pass
            
    def callback(self, msg):
        if self.depth_image is None:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        cv2.imwrite('/home/febintj007/.gemini/antigravity/brain/fa238fa6-ad73-4c9e-90d4-fbe517c03784/latest_cam.jpg', cv_image)
        
        h, w = hsv.shape[:2]
        center_crop = hsv[h//2-20:h//2+20, w//2-20:w//2+20]
        
        print(f"Center 40x40 patch stats:")
        print(f"H min/max: {center_crop[:,:,0].min()} / {center_crop[:,:,0].max()}")
        print(f"S min/max: {center_crop[:,:,1].min()} / {center_crop[:,:,1].max()}")
        print(f"V min/max: {center_crop[:,:,2].min()} / {center_crop[:,:,2].max()}")
        
        # Center depth
        depth_val = self.depth_image[h//2, w//2]
        print(f"Depth at center (y={h//2}, x={w//2}): {depth_val}")
        
        valid_z = not np.isnan(depth_val) and 0.2 < depth_val < 2.0
        print(f"Is depth within valid range (0.2 < z < 2.0)? {valid_z}")
        
        raise SystemExit

def main():
    rclpy.init()
    node = ColorAnalyzer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()

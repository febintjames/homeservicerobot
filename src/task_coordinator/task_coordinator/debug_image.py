#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageCatcher(Node):
    def __init__(self):
        super().__init__('image_catcher')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, 
            '/head_front_camera/rgb/image_raw',
            self.callback, 
            10)
        self.count = 0
            
    def callback(self, msg):
        if self.count >= 1:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        path = '/home/febintj007/.gemini/antigravity/brain/7ebdf5c7-6315-4549-bf20-bdc07ecb8551/robot_view.jpg'
        cv2.imwrite(path, cv_image)
        self.get_logger().info(f'Saved image to {path}')
        self.count += 1
        raise SystemExit

def main():
    rclpy.init()
    node = ImageCatcher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

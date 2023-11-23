#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

from cv2_bridge import CvBridge
import cv2
from math import sin, cos, pi, atan2

BIN_THRESH = 200
AREA_THRESH = 200

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 
        'techman_image', self.image_callback, 10)
    
    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data)

        # process image to get brick centroids
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        gauss_img = cv2.GaussianBlur(gray, (3, 3), 0, 0)

        rt, bin_img = cv2.threshold(gauss_img, BIN_THRESH, 255, cv2.THRESH_BINARY)

        contours, hiearchy = cv2.findContours(bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # a dict to store cx, cy, angle
        blocks = {}

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < AREA_THRESH:
                # too small, probably noise or error
                continue
            M = cv2.moments(cnt)

            # the centroid of the contour
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            # principal angle of the contour
            angle = atan2(2*M["mu11"], M["mu20"]-M["mu02"])/2

            blocks.cx =cx
            blocks.cy = cy
            blocks.angle = angle

        

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

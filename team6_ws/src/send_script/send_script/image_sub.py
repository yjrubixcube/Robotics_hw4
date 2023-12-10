#!/usr/bin/env python
import rclpy
import threading

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
import csv
from math import atan2, sin, cos, pi, tan
# from home.robot.workspace import arm_mode
arm_mode = "C"

imgs=[]
IMG_NUM=10

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 
        'techman_image', self.image_callback, 10)
        self.cur_num=0
    
    def image_callback(self, data):
        self.get_logger().info('Received image')
        print("get img!")
        # TODO (write your code here)
        # name=self.get_name()
        # print(name, self.get_parameter(name))
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data)
        imgs.append(image)
        self.cur_num+=1
        if self.cur_num==IMG_NUM:
            print(calibrate(imgs))

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

def calibrate(imgs):
    print("start calibrate:")
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((8*6,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images=imgs
    img_size=(0,0)
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size=gray.shape[::-1]
        print(img_size)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            print("True")
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
    return mtx

def func():
    for i in range(IMG_NUM):
        send_script("Vision_DoJob(job1)")
        print(f"press any key to get another photo {i+1}/{IMG_NUM}")
    pass

def main(args=None):
    rclpy.init(args=args)
    target_photo = "230.00, 230, 730, -180.00, 0.0, 135.00"
    move_to_photo_pos = "PTP(\"CPP\","+target_photo+",100,200,0,false)"
    send_script(move_to_photo_pos)
    print("c")
    node = ImageSub('image_sub')
    for i in range(IMG_NUM):
        send_script("Vision_DoJob(job1)")
        print(f"press any key to get another photo {i+1}/{IMG_NUM}")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy.linalg import inv
import csv
from math import atan2, sin, cos, pi, tan
# from home.robot.workspace import arm_mode
arm_mode = "C"

CORNER_COUNT = 6, 8
SHRINK_CONST = 1
BIN_THRESH = 200
AREA_THRESH = 200



# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CORNER_COUNT[0]*CORNER_COUNT[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CORNER_COUNT[0],0:CORNER_COUNT[1]].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

calibration_mode = True

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 
        'techman_image', self.image_callback, 10)
    
    def image_callback(self, data):
        global calibration_mode
        self.get_logger().info('Received image')

        # TODO (write your code here)
        # name=self.get_name()
        # print(name, self.get_parameter(name))
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data)
        # pick up
        block_centers=find_blocks(image)
        pickup_blocks(block_centers)


def find_blocks(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_copy=gray.copy()
    gauss_img = cv2.GaussianBlur(gray, (3, 3), 0, 0)

    rt, bin_img = cv2.threshold(gauss_img, BIN_THRESH, 255, cv2.THRESH_BINARY)

    contours, hiearchy = cv2.findContours(bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    block_centers=[]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # perimeter = cv2.arcLength(cnt, True)

        if area < AREA_THRESH:
            continue
        M = cv2.moments(cnt)

        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        angle = atan2(2*M["mu11"], M["mu20"]-M["mu02"])/2
        block_centers.append((cx, cy, angle))
    return block_centers

camera_mtx=np.array([[3.45192788e+03, 0.00000000e+00, 1.31152653e+03],\
                        [0.00000000e+00, 4.07429644e+03, 2.26852567e+03],\
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # get this from calibrate camera
camera_mtx_inv=inv(camera_mtx) # invesing the matrix is required (see the definition of camera matrix)

def camera_navigation(block_center):
    x=block_center[0];y=block_center[1]

    line_vec=camera_mtx_inv@np.array([x,y,1]) # 畫素x,y 只能解出一條射線，這是射線的向量方向（以相機座標系來看）

    #the xyz of the image is not equal to the camera xyz, do transformation here
    line_vec_new=np.array([line_vec[1],-line_vec[0],1])
    line_vec=line_vec_new
    # print(line_vec)

    # camera coordinate to robot base coordinate

    # a,b,c are in degrees
    a=0;b=0;c=0 
    a=-180
    b=0
    c=135

    # so transform to radian
    rada=a*np.pi/180
    radb=b*np.pi/180
    radc=c*np.pi/180

    # you need to choose one of the following
    # abc = xyz euler
    rotation_matrix=\
    np.array([[1,0,0]\
            ,[0,np.cos(rada),-np.sin(rada)]\
            ,[0,np.sin(rada),np.cos(rada)]])*\
    np.array([[np.cos(radb),0,np.sin(radb)]\
            ,[0,1,0]\
            ,[-np.sin(radb),0,np.cos(radb)]])*\
    np.array([[np.cos(radc),-np.sin(radc),0]\
            ,[np.sin(radc),np.cos(radc),0]\
            ,[0,0,1]])

    # rotation_matrix@line_vec is line_vec transformed to the base coordinate
    line_vec_base=rotation_matrix@line_vec

    # check the xyz in the system is camera xyz or the gripper xyz
    # THEY are DIFFERENT!

    # we use camera xyz here
    cam_x=0;cam_y=0;cam_z=0
    cam_x=230
    cam_y=230
    cam_z=730
    cam_pos=np.array([cam_x,cam_y,cam_z])

    # table_z is the z value of the table plane, I set it to 0
    table_z=0

    # 射線公式不是 "點+t倍向量" 嗎? 這裡在解t
    # 因為只跟桌面求解，用z座標就能解t
    t=(table_z-cam_z)/line_vec_base[2]

    # 之後就是代公式了
    block_pos=cam_pos+t*line_vec_base # block_pos就是積木位置 到這裡已經被計算出來
    return block_pos

block_height=30
def pickup_blocks(block_centers):
    target_height=0
    for block_center in block_centers:
        block_pos=camera_navigation(block_center)
        target_block=f"{block_pos[0]}, {block_pos[1]}, 100, 0, {((180/np.pi)*block_center[2])+45}"
        move_to_target_block=f"PTP(\"CPP\",{target_block},100,200,0,false)"
        send_script(move_to_target_block)
        grip(1.0)
        stack_pos=f"400,400,{100+target_height},100,0,135"
        target_height+=block_height
        move_to_stack_pos=f"PTP(\"CPP\",{stack_pos},100,200,0,false)"
        send_script(move_to_stack_pos)
        grip(0.0)
        ## move upwards by 100
        stack_pos_upward=f"400,400,{200+target_height},100,0,135"
        move_upward = "PTP(\"CPP\","+stack_pos_upward+",100,200,0,false)"
        send_script(move_upward)
    
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def grip(state):
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
    target_photo = "230.00, 230, 730, -180.00, 0.0, 135.00"
    move_to_photo_pos = "PTP(\"CPP\","+target_photo+",100,200,0,false)"
    send_script(move_to_photo_pos)
    send_script("Vision_DoJob(job1)")
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

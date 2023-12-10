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
        if calibration_mode:
            # camera calibration mode
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CORNER_COUNT, None)
            # print(ret)
            # print(corners)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                # calibration_mode = False

            else:
                print("calibration failed")
                return
            
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

            # newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,gray.shape[::-1],1,gray.shape[::-1])
        
            dst = cv2.undistort(gray, mtx, dist, None, mtx)
            print(mtx)
            with open("cameraMatrix.csv", 'w', newline='') as file:
                # file.write
                # json.dump(newcameramtx, file)
                writer = csv.writer(file)

                for i in mtx:
                    # for j in i:
                    writer.writerow(list(i))
            cv2.imshow("undis", dst)
            cv2.imshow("og", gray)
            # print("press ESC")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            recalibrate = input("Recalibrate? (y/n): ")
            if recalibrate == "n":
                calibration_mode = False
            elif recalibrate == "y":
                calibration_mode = True
            else:
                calibration_mode = True

        else:
            # pick up
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray_copy=gray.copy()
            gauss_img = cv2.GaussianBlur(gray, (3, 3), 0, 0)

            rt, bin_img = cv2.threshold(gauss_img, BIN_THRESH, 255, cv2.THRESH_BINARY)

            contours, hiearchy = cv2.findContours(bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # perimeter = cv2.arcLength(cnt, True)

                if area < AREA_THRESH:
                    continue
                M = cv2.moments(cnt)

                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                angle = atan2(2*M["mu11"], M["mu20"]-M["mu02"])/2
                print(cx, cy, angle)
            pass
        # cv2.imshow("image", image)
        # cv2.waitKey(0)


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

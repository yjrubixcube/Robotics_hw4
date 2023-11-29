#!/usr/bin/env python

import rclpy
import cv2
import sys
import time
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

# global arm_mode 
arm_mode = "CC"

# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

# gripper client
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

    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, -180.00, 0.0, 135.00"
    targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # targetP2 = "300.00, 100, 500, -180.00, 0.0, 135.00"
    targetP3 = "300.00, 100, 500, 0.00, 0.0, 0.00"
    targetP4 = "300.00, 100, 500, 90.00, 0.0, 0.00"
    targetP5 = "300.00, 100, 500, 90.00, 90.0, 0.00"
    targetP6 = "300.00, 100, 500, 0.00, 90.0, 0.00"
    script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    # script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    script3 = "PTP(\"CPP\","+targetP3+",100,200,0,false)"
    script4 = "PTP(\"CPP\","+targetP4+",100,200,0,false)"
    script5 = "PTP(\"CPP\","+targetP5+",100,200,0,false)"
    script6 = "PTP(\"CPP\","+targetP6+",100,200,0,false)"
    send_script(script1)
    time.sleep(1)
    # send_script(script2)
    # time.sleep(1)
    # send_script(script3)
    # time.sleep(1)
    # send_script(script4)
    # time.sleep(1)
    # send_script(script5)
    # time.sleep(1)
    # send_script(script6)
    # time.sleep(1)

# What does Vision_DoJob do? Try to use it...
# -------------------------------   ------------------
    send_script("Vision_DoJob(CC)")
    cv2.waitKey(1)
    send_script("Vision_DoJob(AA)")
    cv2.waitKey(1)
    # arm_mode = "PICK"
    # for i in range(100):        
    #     send_script("Vision_DoJob(job1)")
#--------------------------------------------------
    
    set_io(0.0) # 1.0: close gripper, 0.0: open gripper
    # set_io(0.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


    

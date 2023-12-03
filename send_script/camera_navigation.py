import cv2
import numpy as np
from numpy.linalg import inv
import csv

x=0;y=0 # x,y are the pixel xy in the image
x=int(input("x: "))   
y=int(input("y: "))

camera_mtx=np.array([[3.45192788e+03, 0.00000000e+00, 1.31152653e+03],\
                    [0.00000000e+00, 4.07429644e+03, 2.26852567e+03],\
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # get this from calibrate camera
file = open("cameraMatrix.csv", 'r', newline='')
reader = csv.reader(file)
# print(reader)
camera_mtx = []
for row in reader:
    camera_mtx.append(row)
camera_mtx = np.array(camera_mtx, dtype=np.float)
print(camera_mtx, type(camera_mtx))
camera_mtx_inv=inv(camera_mtx) # invesing the matrix is required (see the definition of camera matrix)

line_vec=camera_mtx_inv@np.array([x,y,1]) # 畫素x,y 只能解出一條射線，這是射線的向量方向（以相機座標系來看）

#right hand coordinate to left hand coordinate
line_vec_new=np.array([line_vec[1],-line_vec[0],1])
line_vec=line_vec_new
print(line_vec)

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

# abc = xzy euler
# rotation_matrix=\
# np.array([[1,0,0]\
#          ,[0,np.cos(rada),-np.sin(rada)]\
#          ,[0,np.sin(rada),np.cos(rada)]])*\
# np.array([[np.cos(radb),-np.sin(radb),0]\
#          ,[np.sin(radb),np.cos(radb),0]\
#          ,[0,0,1]])*\
# np.array([[np.cos(radc),0,np.sin(radc)]\
#          ,[0,1,0]\
#          ,[-np.sin(radc),0,np.cos(radc)]])

# abc = yxz euler
# rotation_matrix=\
# np.array([[np.cos(rada),0,np.sin(rada)]\
#          ,[0,1,0]\
#          ,[-np.sin(rada),0,np.cos(rada)]])*\
# np.array([[1,0,0]\
#          ,[0,np.cos(radb),-np.sin(radb)]\
#          ,[0,np.sin(radb),np.cos(radb)]])*\
# np.array([[np.cos(radc),-np.sin(radc),0]\
#          ,[np.sin(radc),np.cos(radc),0]\
#          ,[0,0,1]])

# abc = yzx euler
# rotation_matrix=\
# np.array([[np.cos(rada),0,np.sin(rada)]\
#          ,[0,1,0]\
#          ,[-np.sin(rada),0,np.cos(rada)]])*\
# np.array([[np.cos(radb),-np.sin(radb),0]\
#          ,[np.sin(radb),np.cos(radb),0]\
#          ,[0,0,1]])*\
# np.array([[1,0,0]\
#          ,[0,np.cos(radc),-np.sin(radc)]\
#          ,[0,np.sin(radc),np.cos(radc)]])

# abc = zyx euler
# rotation_matrix=\
# np.array([[np.cos(rada),-np.sin(rada),0]\
#          ,[np.sin(rada),np.cos(rada),0]\
#          ,[0,0,1]])*\
# np.array([[np.cos(radb),0,np.sin(radb)]\
#          ,[0,1,0]\
#          ,[-np.sin(radb),0,np.cos(radb)]])*\
# np.array([[1,0,0]\
#          ,[0,np.cos(radc),-np.sin(radc)]\
#          ,[0,np.sin(radc),np.cos(radc)]])

# abc = zxy euler
# rotation_matrix=\
# np.array([[np.cos(rada),-np.sin(rada),0]\
#          ,[np.sin(rada),np.cos(rada),0]\
#          ,[0,0,1]])*\
# np.array([[1,0,0]\
#          ,[0,np.cos(radb),-np.sin(radb)]\
#          ,[0,np.sin(radb),np.cos(radb)]])*\
# np.array([[np.cos(radc),0,np.sin(radc)]\
#          ,[0,1,0]\
#          ,[-np.sin(radc),0,np.cos(radc)]])

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
table_z=100

# 射線公式不是 "點+t倍向量" 嗎? 這裡在解t
# 因為只跟桌面求解，用z座標就能解t
t=(table_z-cam_z)/line_vec_base[2]

# 之後就是代公式了
block_pos=cam_pos+t*line_vec_base # block_pos就是積木位置 到這裡已經被計算出來
# 110, 80
print("the block position is:")
print(block_pos)
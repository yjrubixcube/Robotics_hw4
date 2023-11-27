import cv2
import numpy as np
from numpy.linalg import inv

x=0;y=0 # x,y are the pixel xy in the image

camera_mtx=np.ndarray((3,3)) # get this from calibrate camera

camera_mtx_inv=inv(camera_mtx) # invesing the matrix is required (see the definition of camera matrix)

line_vec=camera_mtx_inv@np.transpose(np.array([x,y,1])) # 畫素x,y 只能解出一條射線，這是射線的向量方向（以相機座標系來看）

# camera coordinate to robot base coordinate
# you need to choose one of the following

# a,b,c are in degrees
a=0;b=0;c=0 

# so transform to radian
rada=a*np.pi/180
radb=b*np.pi/180
radc=c*np.pi/180

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
rotation_matrix=\
np.array([[1,0,0]\
         ,[0,np.cos(rada),-np.sin(rada)]\
         ,[0,np.sin(rada),np.cos(rada)]])*\
np.array([[np.cos(radb),-np.sin(radb),0]\
         ,[np.sin(radb),np.cos(radb),0]\
         ,[0,0,1]])*\
np.array([[np.cos(radc),0,np.sin(radc)]\
         ,[0,1,0]\
         ,[-np.sin(radc),0,np.cos(radc)]])

# abc = yxz euler
rotation_matrix=\
np.array([[np.cos(rada),0,np.sin(rada)]\
         ,[0,1,0]\
         ,[-np.sin(rada),0,np.cos(rada)]])*\
np.array([[1,0,0]\
         ,[0,np.cos(radb),-np.sin(radb)]\
         ,[0,np.sin(radb),np.cos(radb)]])*\
np.array([[np.cos(radc),-np.sin(radc),0]\
         ,[np.sin(radc),np.cos(radc),0]\
         ,[0,0,1]])

# abc = yzx euler
rotation_matrix=\
np.array([[np.cos(rada),0,np.sin(rada)]\
         ,[0,1,0]\
         ,[-np.sin(rada),0,np.cos(rada)]])*\
np.array([[np.cos(radb),-np.sin(radb),0]\
         ,[np.sin(radb),np.cos(radb),0]\
         ,[0,0,1]])*\
np.array([[1,0,0]\
         ,[0,np.cos(radc),-np.sin(radc)]\
         ,[0,np.sin(radc),np.cos(radc)]])

# abc = zyx euler
rotation_matrix=\
np.array([[np.cos(rada),-np.sin(rada),0]\
         ,[np.sin(rada),np.cos(rada),0]\
         ,[0,0,1]])*\
np.array([[np.cos(radb),0,np.sin(radb)]\
         ,[0,1,0]\
         ,[-np.sin(radb),0,np.cos(radb)]])*\
np.array([[1,0,0]\
         ,[0,np.cos(radc),-np.sin(radc)]\
         ,[0,np.sin(radc),np.cos(radc)]])

# abc = zxy euler
rotation_matrix=\
np.array([[np.cos(rada),-np.sin(rada),0]\
         ,[np.sin(rada),np.cos(rada),0]\
         ,[0,0,1]])*\
np.array([[1,0,0]\
         ,[0,np.cos(radb),-np.sin(radb)]\
         ,[0,np.sin(radb),np.cos(radb)]])*\
np.array([[np.cos(radc),0,np.sin(radc)]\
         ,[0,1,0]\
         ,[-np.sin(radc),0,np.cos(radc)]])



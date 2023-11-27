import cv2
import numpy as np
from numpy.linalg import inv

x=0;y=0 # x,y are the pixel xy in the image

camera_mtx=np.ndarray((3,3)) # get this from calibrate camera

camera_mtx_inv=inv(camera_mtx) # invesing the matrix is required (see the definition of camera matrix)

line_vec=camera_mtx_inv@np.array([x,y,1]) # 畫素x,y 只能解出一條射線，這是射線的向量方向（以相機座標系來看）

# camera coordinate to robot base coordinate
# you need to choose one of the following

cam_x=0;cam_y=0;cam_z=0

rotation_matrix=np.array([[0,0,0,cam_x],[0,0,0,cam_y],[0,0,0,cam_z],[0,0,0,1]])
print(rotation_matrix)
# abc = xyz euler

# abc = xzy euler

# abc = yxz euler

# abc = yzx euler

# abc = zyx euler

# abc = zxy euler



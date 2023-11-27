import cv2
import numpy as np
from numpy.linalg import inv

x=0;y=0 # x,y are the pixel xy in the image

camera_mtx=np.array(3,3) # get this from calibrate camera

camera_mtx_inv=inv(camera_mtx) # invesing the matrix is required (see the definition of camera matrix)

line_vec=camera_mtx_inv@np.ndarray([x,y,1]) # 畫素x,y 只能解出一條射線，這是射線的向量方向（以相機座標系來看）

rotation_matrix=np.array(4,4) # I don't know the meaning of abc, so 


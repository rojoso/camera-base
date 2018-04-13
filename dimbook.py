import homography
import camera
import sift
import draw_cube
import scipy.linalg
from numpy import *

from PIL import Image
from pylab import *

sift.process_image('book_frontal.jpg','im0.sift')

l0,d0 = sift.read_features_file('im0.sift')

sift.process_image('book_perspectial.jpg','im1.sift')

l1,d1 = sift.read_features_file('im1.sift')

#匹配特征，并计算单应性矩阵

matches = sift.match_twoside(d0,d1)

ndx = matches.nonzero()[0]

fp = homography.make_homog(l0[ndx,:2].T)
ndx2 = [int(matches[i]) for i in ndx]

tp = homography.make_homog(l1[ndx2,:2].T)

model = homography.RansacModel()

H = homography.H_from_ransac(fp,tp,model)[0]

#计算照相机标定矩阵

K = camera.Camera.my_calibration((405,300))

box = draw_cube.cube_points([0,0,0.1],0.1)

#投影第一幅图像上底部的正方形

cam1 = camera.Camera(hstack((K,dot(K,array([[0],[0],[-1]])))))

#底部正方形上的点
box_cam1 = cam1.project(homography.make_homog(box[:,:5]))

#使用H 将点变换到第二幅图像中
print(H)

box_trans = homography.normalize1(dot(H,box_cam1))

# 从cam1 和H 中计算第二个照相机矩阵
#此处不是太懂
cam2 = camera.Camera(dot(H,cam1.P))

A = dot(linalg.inv(K),cam2.P[:,:3])
A = array([A[:,0],A[:,1],cross(A[:,0],A[:,1])]).T

cam2.P[:,:3] = dot(K,A)

#使用第二个照相机矩阵进行投影

box_cam2 = cam2.project(homography.make_homog(box))

#册数

points = array([1,1,0,1]).T
print(homography.normalize1(dot(dot(H,cam1.P),points)))

print(cam2.project(points))

im0 = array(Image.open('book_frontal.jpg'))

im1 = array(Image.open('book_perspectial.jpg'))

#底部正方形的二维投影

figure()
imshow(im0)
plot(box_cam1[0,:],box_cam1[1,:],linewidth = 3)

#使用H 对二维投影进行变换

figure()
imshow(im1)
plot(box_trans[0,:],box_trans[1,:],linewidth = 3)

#三维立方体

figure()

imshow(im1)
plot(box_cam2[0,:],box_cam2[1,:],linewidth = 3)

show()

import pickle

with open('ar_camera.pkl','wb') as f:
	pickle.dump(K,f)
	pickle.dump(dot(linalg.inv(K),cam2.P),f)



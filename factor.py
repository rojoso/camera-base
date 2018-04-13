from numpy import *
import scipy
import camera

K = array([[1000,0,500],[0,1000,300],[0,0,1]])

tmp = camera.Camera.rotation_matrix([0,0,1])[:3,:3]

Rt = hstack((tmp,array([[50],[40],[30]])))

cam = camera.Camera(dot(K,Rt))

print(K,Rt)
print(cam.factor())


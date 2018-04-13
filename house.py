import camera 
import numpy
from numpy import *
from pylab import *
import scipy
from PIL import Image

#最点进行载入

points = loadtxt('./house/house.p3d').T

points = vstack((points,ones(points.shape[1])))

#设置照相机参数

P = hstack((eye(3),array([[0],[0],[-10]])))

cam = camera.Camera(P)
x = cam.project(points)


#创建变换

r = 0.05 * numpy.random.rand(3)
rot = camera.Camera.rotation_matrix(r)
#绘制投影

figure()

for t in range(20):
	cam.P = dot(cam.P,rot)
	x = cam.project(points)
	plot(x[0],x[1],'k.')
show()


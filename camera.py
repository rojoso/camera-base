
import scipy.linalg
from numpy import *

class Camera(object):
	'''表示针孔照相机的类'''

	def __init__(self,P):
		'''对照相机进行初始化'''
		self.P = P
		self.K = None #标定矩阵，即内参数
		self.R = None #旋转 外参数之一

		self.t = None #平移
		self.c = None #照相机中心

	def project(self,X):
		''' X (4*n的数组)  的投影点，并且进行坐标的归一化
		'''
		x = dot(self.P,X)
		for i in range(3):
			x[i]/=x[2]

		return x

	def rotation_matrix(a):
		''' 创建一个用于围绕向量a轴旋转的三维旋转矩阵'''

		R = eye(4)
		R[:3,:3] = scipy.linalg.expm([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])

		return R

	def factor(self):
		''' 将照相机矩阵分解为K,R，t，其中，P = K[R|t]
		'''
		#分解前3*3的部分
		K,R = scipy.linalg.rq(self.P[:3,:3])

		#将K的对角线元素均设置为正值
		T = diag(sign(diag(K)))
		if linalg.det(T) <0:
			T[1,1] *= -1

		self.K = dot(K,T)
		self.R = dot(T,R) # T的逆矩阵为其自身

		self.t = dot(linalg.inv(self.K),self.P[:,3])
		return self.K,self.R,self.t

	def center(self):
		''' 计算并返回照相机的中心店'''
		if self.c is not None:
			return self.c
		else:
			#通过因子分解计算c
			self.factor()
			self.c = -dot(self.R.T,self.t)

			return self.c

	def my_calibration(sz):
		row,col = sz
		fx = 500*col/405
		fy = 550*row/312
		K = diag([fx,fy,1])
		K[0,2] = 0.5*col
		K[1,2] = 0.5*row

		return K








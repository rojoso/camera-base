from pylab import *
import matplotlib.pyplot as plt

def cube_points(c,wid):
	''' 创建用于绘制立方体的一个点列表'''
	p = []
	#底部
	p.append([c[0]-wid,c[1]-wid,c[2]-wid])
	p.append([c[0]-wid,c[1]+wid,c[2]-wid])
	p.append([c[0]+wid,c[1]+wid,c[2]-wid])
	p.append([c[0]+wid,c[1]-wid,c[2]-wid])
	p.append([c[0]-wid,c[1]-wid,c[2]-wid])

	#顶部
	p.append([c[0]-wid,c[1]-wid,c[2]+wid])
	p.append([c[0]-wid,c[1]+wid,c[2]+wid])
	p.append([c[0]+wid,c[1]+wid,c[2]+wid])
	p.append([c[0]-wid,c[1]-wid,c[2]+wid])
	p.append([c[0]-wid,c[1]-wid,c[2]+wid])

	#竖直的边
	p.append([c[0]-wid,c[1]-wid,c[2]+wid])
	p.append([c[0]-wid,c[1]+wid,c[2]+wid])
	p.append([c[0]-wid,c[1]+wid,c[2]-wid])
	p.append([c[0]+wid,c[1]+wid,c[2]-wid])
	p.append([c[0]+wid,c[1]+wid,c[2]+wid])
	p.append([c[0]+wid,c[1]-wid,c[2]+wid])
	p.append([c[0]+wid,c[1]-wid,c[2]-wid])

	return array(p).T

'''
poi = cube_points([0,0,0.1],0.1)

figure()
plot(poi[0,:],poi[1,:],linewidth = 3)

show()


'''












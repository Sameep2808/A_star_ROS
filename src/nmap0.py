import os
import math
import numpy as np
from numpy import array
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation 

fig, ax = plt.subplots()

ymax = 999
xmax = 999

def eq(x1,y1,x2,y2,x,y,f):
	m = (y2-y1)/(x2-x1)
	if (f == 1):
		c = (m*x) - y <= (m*x1) - y1 
	else:
		c = (m*x) - y >= (m*x1) - y1 
	return c

def create_map():
	m = np.zeros((1000,1000))
	am = np.zeros((1000,1000,3))
	hl = 40.4145
	for y in range(m.shape[0]):
		for x in range(m.shape[1]):
			if (((y - 800) ** 2) + ((x - 200) ** 2) <= ((100) ** 2) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if (((y - 200) ** 2) + ((x - 200) ** 2) <= ((100) ** 2) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ( (x >= 25) and (x <= 175) and (y >= 425) and (y <= 575) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ( (x >= 375) and (x <= 625) and (y >= 425) and (y <= 575) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ( (x >= 725) and (x <= 875) and (y >= 600) and (y <= 800) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			# if ((x > 200-35) and (x < 200 + 35) and (y <= 150) and eq(200,150-hl,165,150-(hl/2),x,y,1) and eq(200,150-hl,235,150-(hl/2),x,y,1) ):
			# 	m[y,x]=1
			# 	am[y,x]=[255,0,0]
			# if ((x > 200-35) and (x < 200 + 35) and (y >= 150) and eq(200,150+hl,165,150+(hl/2),x,y,2) and eq(200,150+hl,235,150+(hl/2),x,y,2) ):
			# 	m[y,x]=1
			# 	am[y,x]=[255,0,0]
			# if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(80,70,105,150,x,y,1) and (y >= 1000-180))):
			# 	m[y,x]=1
			# 	am[y,x]=[255,0,0]
			# if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(115,40,80,70,x,y,2) and (y <= 1000-180))):
			# 	m[y,x]=1
			# 	am[y,x]=[255,0,0]
	return m,am


def detect(m, am, x,y):
	global cll
	for cl in range(0,cll):
		if (1000 - (y+cl) > 0):
			if (m[ymax-(y+cl)][x] == 1):
				#print("1")
				return True
		if (1000 - (y-cl) <= ymax):
			if (m[ymax-(y-cl)][x] == 1):
				#print("2")
				return True
		if ( x+cl < xmax ):
			if (m[ymax-y][x+cl] == 1):
				#print("3")
				return True
		if ( x-cl > 0 ):
			if (m[ymax-y][x-cl] == 1):
				#print("4")
				return True
		if (1000 - (y+cl) > 0) and ( x+cl < xmax ):
			if (m[ymax-(y+cl)][x+cl] == 1):
				#print("1")
				return True
		if (1000 - (y+cl) > 0) and ( x-cl > 0 ):
			if (m[ymax-(y+cl)][x-cl] == 1):
				#print("2")
				return True
		if (1000 - (y-cl) <= ymax) and ( x+cl < xmax ):
			if (m[ymax-(y-cl)][x+cl] == 1):
				#print("2")
				return True
		if (1000 - (y-cl) <= ymax) and ( x-cl > 0 ):
			if (m[ymax-(y-cl)][x-cl] == 1):
				#print("2")
				return True
	global rr
	#print(rr)
	if (1000 - (y+rr) < 0) or (1000 - (y-rr)  >= ymax) or ( x-rr < 0 ) or ( x+rr > xmax ):
		return True
	return False


class Node:
	def __init__(self, data, orientaion, cost, parent, gcost, pxy):
		self.d = data
		self.o = orientaion
		self.c = cost
		self.p = parent
		self.g = gcost
		self.pxy = pxy


def action(goal, CurrentNode,UL,UR,m,am):
	x,y = CurrentNode.d
	# thetat = theta + CurrentNode.o
	# xd = L*math.cos((math.pi/180)*thetat)
	# yd = L*math.sin((math.pi/180)*thetat)
	xd,yd,theta,gc = plot_curve(x,y,CurrentNode.o,UL,UR)
	if ((1000-round(yd)) > 0) and (round(xd) < xmax) and ((1000-round(yd)) <= 1000) and (round(xd) > 0):
		if (m[ymax-round(yd)][round(xd)] == 1):
			return None
		if (detect(m, am, round(xd),round(yd))):
			#print("Here")
			return None
		else:
			cost = CurrentNode.c
			gc = getcost(goal, xd,yd)
			orientation = theta
			if(orientation >= 360):
				orientation = orientation - 360
			if(orientation < 0):
				orientation = orientation + 360
			# child_node = Node([(x+xd),(y+yd)],orientation,cost+1.4,CurrentNode,gc,CurrentNode.d)
			child_node = Node([(xd),(yd)],orientation,cost+1.4,CurrentNode,gc,CurrentNode.d)
			return child_node
	else:
		return None 

def plot_curve(Xn,Yn,Thetai,UL,UR):
	t = 0
	r = 0.33 
	L = 28.7 
	dt = 0.1
	# Xn=Xi
	# Yn=Yi
	Thetan = 3.14 * Thetai / 180
	D=0
	while t<1:
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
		Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
		Thetan += (r / L) * (UR - UL) * dt
		D += (math.pow(Xn,2)+math.pow(Yn,2))
		plt.plot([Xs, Xn], [Ys, Yn], color="blue")
		
	Thetan = 180 * (Thetan) / 3.14
	
	return Xn, Yn, Thetan, D

def move(goal, direction, node, L,m,am):
	UL,UR = direction
	return action(goal, node,UL,UR,m,am)
	
# def move(direction, node, L):
# 	if direction == 1:
# 		return action(node,L,60)
# 	elif direction == 2:
# 		return action(node,L,30)
# 	elif direction == 3:
# 		return action(node,L,0)
# 	elif direction == 4:
# 		return action(node,L,-30)
# 	elif direction == 5:
# 		return action(node,L,-60)
# 	elif direction == 6:
# 		return action(node,L,0)

def DS(node, goal, L,ga,m,am):
	Q = [node]
	CL = []
	OL,cOL,OLc,P = [],[],[],[]
	OL.append(node.d)
	OLc.append(node.d)
	cOL.append(node.c)
	P.append(node.pxy)
	# action_set=[[5,5], [5,0],[0,5],[5,10],[10,5],[10,10],[0,10],[10,0]]
	action_set=[[50,50], [50,0],[0,50],[50,100],[100,50],[100,100],[0,100],[100,0]]
	p=0
	while Q:
		l = cOL.index(min(cOL))
		cn = Q.pop(l)
		o=cOL.pop(l)
		o=OL.pop(l)
		ox,oy=o
		CL.append(cn.d)
		print(cn.d[0],cn.d[1])
		# am[ymax-round(cn.d[1])][round(cn.d[0])] = [255,255,255]
		# cv2.arrowedLine(am, (round(cn.pxy[0]), round(ymax-cn.pxy[1])), (round(cn.d[0]), round(ymax-cn.d[1])),(255,255,255), 1, 1, 0, 0.1)
		# out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
		if (((cn.d[1] - goal[1]) ** 2) + ((cn.d[0] - goal[0]) ** 2) <= ((3*L) ** 2)):
			return cn,CL,OLc,P
		for a in action_set:
			NewNode = move(goal, a,cn,L,m,am) 
			if NewNode != None:
				if (NewNode.d not in CL):
					if(NewNode.d not in OL):
						Q.append(NewNode)
						OL.append(NewNode.d)
						OLc.append(NewNode.d)
						P.append(NewNode.pxy)
						cOL.append(NewNode.g)
					else:
						T = Q[OL.index(NewNode.d)]
						if(T.g > NewNode.g):
							T.p = NewNode.p
							T.pxy = NewNode.pxy




def reverse_path(node,m,am):
	path = []
	path = [node]
	c=0
	while node.p != None:
		x,y = node.d
		c=c+1
		m[ymax-round(y)][round(x)] = 1
		node = node.p
		path.append(node)
	path.reverse()
	return path


def Vi(P,C):
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	out = cv2.VideoWriter("Output.mp4", fourcc, 1500, (1000,1000))
	for x,y in C:
		am[ymax-round(y)][round(x)] = [255,255,255]
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	for n in P:
		x,y = n.d 
		am[ymax-round(y)][round(x)] = [0,0,255]
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	cv2.imshow("Final Path", am)
	cv2.waitKey(0)
	out.release()
	cv2.destroyAllWindows()


def getcost(goal, x,y):
	xg,yg = goal
	ec = (x-xg)**2 + (y-yg)**2
	return ec

# def get_input():
# 	global cll
# 	global rr
# 	cll = 5
# 	rr = 5
# 	cll = cll + rr  
	
	
		
# 	return [cll,cll],[200-cll,cll],5,0,0

def get_input(m,am):
	global cll
	global rr
	rr = 22
	cll = 10
	L = 10
	# print('Enter Clearence:')
	# cll = int(input())
	# print('Enter Robot Radius:')
	# rr = int(input())
	cll = cll + rr  
	# print('Enter movement length:')
	# L = int(input())
	
	# print('Enter Initial X (Range: 0 - xmax):')
	# x = int(input())
	# if (x - rr <0) or (x + rr >xmax):
	# 	print('INVALID X SETTING INITIAL X AS LEAST RADIUS')
	# 	x=rr
		
	# print('Enter Initial Y (Range: 0 - ymax):')
	# y = int(input())
	# if (y - rr <0) or (y +rr >ymax):
	# 	print('INVALID Y SETTING INITIAL Y AS LEAST RADIUS')
	# 	y=rr
	# print('Enter Start Angle:')
	# sa = int(input())
	
	# if detect(m,am,x,y):
	# 	print('INVALID POINTS SETTING INITIAL POINT AS [0,0]')
	# 	x=rr
	# 	y=rr
	x = 50
	y = 50
	sa = 0
		
	print('Enter Goal X (Range: 0 - xmax):')
	xg = int(input())
	if (xg - rr <0) or (xg + rr >xmax):
		print('INVALID X SETTING GOAL X AS xmax')
		xg=xmax - rr
		
	print('Enter Goal Y (Range: 0 - ymax):')
	yg = int(input())
	if (yg - rr <0) or (yg + rr>ymax):
		print('INVALID Y SETTING GOAL Y AS ymax')
		yg=ymax - rr 
	
	if detect(m,am,xg,yg):
		print('INVALID POINTS SETTING INITIAL POINT AS [xmax,ymax]')
		xg=xmax - rr
		yg=ymax - rr
	print('Enter Goal Angle:')
	ga = int(input())
		
	return [x,y],[xg,yg],L,sa,ga


def graph(O,OP,P):
	fig, ax = plt.subplots()
	plt.grid()
	plt.xlim(0,400)
	plt.ylim(0,300)
	
	for c,p in zip(O,OP):
		x,y = c
		px,py = p
		ax.quiver(px,py,x-px,y-py,units='xy' ,scale=1, width = 0.5,headwidth = 10,headlength = 10)
	for z in P:
		x,y = z.d
		px,py = z.pxy
		ax.quiver(px,py,x-px,y-py,units='xy' ,color= 'r',scale=1,headwidth = 10,headlength = 10)
	ax.set_aspect('equal')
	plt.show()


def Vig(O,OP,P,m,am):
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	out = cv2.VideoWriter("Output.mp4", fourcc, 10, (xmax,ymax))

	for c,p in zip(O,OP):
		x,y = c
		px,py = p
		cv2.arrowedLine(am, (round(px), round(ymax-py)), (round(x), round(ymax-y)),(255,255,255), 1, 1, 0, 0.1)
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	for z in P:
		x,y = z.d
		px,py = z.pxy
		cv2.arrowedLine(am, (round(px), round(ymax-py)), (round(x), round(ymax-y)),(255,0,255), 1, 1, 0, 0.1)
		out.write(np.uint8(am))
		cv2.imshow("Output", am)
		cv2.waitKey(1)
	# cv2.imshow("Final Path", am)
	# cv2.waitKey(0)
	out.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	m,am = create_map()
	global cll
	global rr
	start = []
	global goal
	start,goal,L,sa,ga=get_input(m,am)
	root = Node(start, sa, 0 , None, 0, start)
	F,C,O,Pxy = DS(root,goal,L,ga,m,am)
	p=reverse_path(F,m,am)
	Vig(O,Pxy,p,m,am)
	plt.grid()

	ax.set_aspect('equal')

	# plt.xlim(0,1)
	# plt.ylim(0,1)

	plt.title('How to plot a vector in matplotlib ?',fontsize=10)

	plt.show()
	plt.close()

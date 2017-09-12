import numpy as np
#import cv2
import time
import os
import ikpy as ik
from ikpy import plot_utils

g = 9.8
l = 0.23
A = 10
f = (1/(2*np.pi))*np.sqrt(g/l)
w = 2*np.pi*f
t = 0
start = time.time()
#perna - quadril = target
link0 = ik.link.URDFLink("calc_lateral", [0,0,1.9], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds=(-30,30))
link1 = ik.link.URDFLink("calc_frontal", [0,0,0.7], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-60,60))
link2 = ik.link.URDFLink("joelho", [0,0,8.2] , [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-60,60))
link3 = ik.link.URDFLink("quadril", [0,0,6.4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-80,80))
link4 = ik.link.URDFLink("pelves", [0, 1.7, 4], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True, bounds=(-50,50))

#chains
foot2pelv = ik.chain.Chain([link0, link1, link2, link3, link4], [True, True, True, False, False])

#start joint positions
joints = np.deg2rad([0, 13.24660153,-30.31297739,17.0563224,0])


target = [4., 1.7, 20.7]
frame_target = np.eye(4)
frame_target[:3, 3] = target
ik = foot2pelv.inverse_kinematics(frame_target,initial_position=joints)


def plot_chain (corrente, juntas=None, alvo=None):
	if juntas is None:
		juntas = [0] * len(corrente.links)
	ax = plot_utils.init_3d_figure()
	corrente.plot(juntas, ax, target=alvo)
	plot_utils.show_figure()


def calculaSinalY(tempo):
	Y = A*(1 + (l/g)*w*w)*np.sin(w*tempo)
	return Y



#SETUP
print(np.rad2deg(ik))
plot_chain(foot2pelv, juntas=ik, alvo=target)

#LOOP
while 0:
	#timer between frames
	dTime = time.time() - start
	start = time.time()
	t += dTime
	Y = calculaSinalY(t)
	os.system("clear")
	print (Y)
	time.sleep(0.05)
	

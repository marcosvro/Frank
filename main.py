import numpy as np
import cv2
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
#origin = ik.link.OriginLink()
link1 = ik.link.URDFLink("pe", [0, 0, 0], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link2 = ik.link.URDFLink("quadril", [5, 0, 10] , [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link3 = ik.link.URDFLink("head", [-5, 0, 13], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
#link2 = ik.link.URDFLink("spine", d=2, a=0, use_symbolic_matrix=True)
arm = ik.chain.Chain([link1, link2, link3], [True, True, True])
joints = [0] * len(arm.links)
target = [0, 0, 15] #altetar o parametro Z para subir o pé
frame_target = np.eye(4)
frame_target[:3, 3] = target
ik = arm.inverse_kinematics(frame_target,initial_position=joints)
ax = plot_utils.init_3d_figure()
arm.plot(ik, ax, target=target)
#plot_utils.plot_chain(arm, joints, ax)
plot_utils.show_figure()


while 0:
	dTime = time.time() - start
	start = time.time()
	t += dTime
	joints[1] = t;
	ax = plot_utils.init_3d_figure()
	plot_utils.plot_chain(arm, joints, ax)
	plot_utils.show_figure()
	Y = A*(1 + (l/g)*w*w)*np.sin(w*t)
	os.system("clear")
	#print (Y)
	time.sleep(0.05)
	

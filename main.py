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
#perna esquerda
link1 = ik.link.URDFLink("pe_esq", [0, -3, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
link2 = ik.link.URDFLink("joelho_esq", [5, 0, 10] , [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link3 = ik.link.URDFLink("coxa_esq", [-5, 0, 13], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link4 = ik.link.URDFLink("quadril_esq", [0, 0, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
#perna direita
link5 = ik.link.URDFLink("pe_dir", [0, 3, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
link6 = ik.link.URDFLink("joelho_dir", [5, 0, 10] , [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link7 = ik.link.URDFLink("coxa_dir", [-5, 0, 13], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link8 = ik.link.URDFLink("quadril_dir", [0, 0, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
#centro de massa
link9 = ik.link.URDFLink("Pelv_esq", [0, 0, 23], [0, 0, 0], [0, 0, 1], use_symbolic_matrix=True)
#chains
perna_esq = ik.chain.Chain([link1, link2, link3, link4], [True, True, True, True])
perna_dir = ik.chain.Chain([link5, link6, link7, link8], [True, True, True, True])

joints = [0] * len(perna_esq.links)
joints2 = [0] * len(perna_dir.links)
target = [0, 3, 23] #altetar o parametro Z para subir o pé
frame_target = np.eye(4)
frame_target[:3, 3] = target
ik = perna_esq.inverse_kinematics(frame_target,initial_position=joints)
ik2 = perna_dir.inverse_kinematics(frame_target,initial_position=joints2)
ax = plot_utils.init_3d_figure()
perna_esq.plot(ik, ax, target=target)
perna_dir.plot(ik2, ax, target=target)
print(np.rad2deg(ik))
plot_utils.show_figure()


def calculaSinalY(tempo):
	Y = A*(1 + (l/g)*w*w)*np.sin(w*tempo)
	return Y





#LOOP
while 1:
	#timer between frames
	dTime = time.time() - start
	start = time.time()
	t += dTime
	Y = calculaSinalY(t)
	os.system("clear")
	print (Y)
	time.sleep(0.05)
	

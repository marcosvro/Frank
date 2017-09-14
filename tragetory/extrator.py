import numpy as np
import threading
#import cv2
import time
import os
import ikpy as ik
from ikpy import plot_utils
import spidev


deslocamentoXpelves = 13.
periodo = 3.
nEstados = 1000
dMovx = deslocamentoXpelves/nEstados
frameRate = periodo/nEstados
data = np.zeros((nEstados,8), dtype=np.int8)


g = 9.8
l = 0.23
A = 10
f = (1/(2*np.pi))*np.sqrt(g/l)
w = 2*np.pi*f 
t = 0


#perna - quadril = target
link0 = ik.link.URDFLink("calc_lateral", [0,0,1.9], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds=(-30,30))
link1 = ik.link.URDFLink("calc_frontal", [0,0,0.7], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-60,60))
link2 = ik.link.URDFLink("joelho", [0,0,8.2] , [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-60,60))
link3 = ik.link.URDFLink("quadril", [0,0,6.4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds=(-80,80))
link4 = ik.link.URDFLink("pelves", [0, 1.7, 4], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True, bounds=(-50,50))

#perna - pé = target
link5 = ik.link.URDFLink("pelves", [0,0,0], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-50, 50))
link6 = ik.link.URDFLink("quadril", [0,-1.7,-4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-80, 80))
link7 = ik.link.URDFLink("joelho", [0,0,-6.4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-60, 60))
link8 = ik.link.URDFLink("calc_frontal", [0,0,-8.2], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-60, 60))
link9 = ik.link.URDFLink("calc_lateral", [0,0,-0.7], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-30, 30))
link10 = ik.link.URDFLink("pé", [0,0,-0.7], [0,0,0], [0,0,0], use_symbolic_matrix=True)


#chains
foot2pelv = ik.chain.Chain([link0, link1, link2, link3, link4], [True, True, True, False, False])
pelv2foot = ik.chain.Chain([link5, link6, link7, link8, link9, link10], [True, True, True, True, False, False])


#start joint positions
jointsf2p = np.deg2rad([0, 13.24660153,-30.31297739,17.0563224,0])
jointsp2f = np.deg2rad([0,17.0563224,-30.31297739,13.24660153,0,0])


#start target position
pos_fix = [0, 1.7, 20.7]
pos_inicial = [-(deslocamentoXpelves/2), 1.7, 20.7]


frame_target = np.eye(4)
frame_target[:3, 3] = pos_inicial
ik = foot2pelv.inverse_kinematics(frame_target,initial_position=jointsf2p)

#comunicacao
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 16000

#FUNÇÕES
def thread_cinematica(indice):
	#calcula cinematica inversa
	pos = pos_inicial
	pos[0] = pos[0] + indice*dMovx
	frame_target = np.eye(4)
	frame_target[:3, 3] = pos
	ik = foot2pelv.inverse_kinematics(frame_target,initial_position=jointsf2p)
	ik = np.rad2deg(ik)
	ik = ik.astype(np.int8)
	#salva dados no array tragetoria
	#calc_lateral	
	data[indice][0] = ik[0] 
	#calc_frontal
	data[indice][1] = ik[1] 
	#joelho
	data[indice][2] = ik[2] 
	#quadril
	data[indice][3] = ik[3] 
	#pelves
	data[indice][4] = ik[4]
	#torso 
	data[indice][5] = 0 
	#braco
	data[indice][6] = 0 	
	#cotovelo	
	data[indice][7] = 0

def calculaTragetoria():
	i = 0
	while i < nEstados:
		#cria threads para calcular cinematica invesa
		thread = threading.Thread(target=thread_cinematica, args=(i, ))
		thread.daemon=True
		thread.start()
		thread.join()
		i += 1;
	
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
#print(np.rad2deg(ik))
#plot_chain(foot2pelv, juntas=ik, alvo=pos_inicial)
print "Calculando tragetoria.. "
calculaTragetoria()
while threading.active_count() != 1:
	os.system("clear")
	print "Calculando tragetoria.. (", threading.active_count(),"/",nEstados,")"


#LOOP
start = time.time()
t = 0.
t_fps = 0.
t_state = 0.
state = 0
fps = 0

while 1:
	#timers
	dTime = time.time() - start
	start = time.time()
	t += dTime
	t_fps += dTime
	t_state += dTime

	#change state
	if(t_state >= frameRate):
		t_state = 0
		state = (state+1)%nEstados

	#fps calculator
	if t_fps > 1:
		os.system("clear")
		print "fps:", fps
		t_fps = 0
		fps = 0
	fps += 1
	
	#sending data
	spi.writebytes(data[state].tolist())
	#print (data[state])
	#time.sleep(0.01)

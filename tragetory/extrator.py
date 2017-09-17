import numpy as np
import threading
#import cv2
import time
import os
import ikpy as ik
from ikpy import plot_utils
import spidev



#CONFIGS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
deslocamentoZpes = 5.
deslocamentoXpes = 13.
deslocamentoYpelves = 4.
periodo = 3.
nEstados = 1000
dMovx = deslocamentoXpes/nEstados
frameRate = periodo/nEstados
data_foot = np.zeros((nEstados,8), dtype=np.int8)
data_pelv = np.zeros((nEstados,8), dtype=np.int8)

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

#perna - pe = target
link5 = ik.link.URDFLink("pelves", [0,0,0], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-50, 50))
link6 = ik.link.URDFLink("quadril", [0,-1.7,-4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-80, 80))
link7 = ik.link.URDFLink("joelho", [0,0,-6.4], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-60, 60))
link8 = ik.link.URDFLink("calc_frontal", [0,0,-8.2], [0,0,0], [0,1,0], use_symbolic_matrix=True, bounds = (-60, 60))
link9 = ik.link.URDFLink("calc_lateral", [0,0,-0.7], [0,0,0], [1,0,0], use_symbolic_matrix=True, bounds = (-30, 30))
link10 = ik.link.URDFLink("pe", [0,0,-0.7], [0,0,0], [0,0,0], use_symbolic_matrix=True)


#chains
foot2pelv = ik.chain.Chain([link0, link1, link2, link3, link4], [True, True, True, False, False])
pelv2foot = ik.chain.Chain([link5, link6, link7, link8, link9, link10], [True, True, True, True, False, False])


#start joint positions
jointsf2p = np.deg2rad([0, 13.24660153,-30.31297739,17.0563224,0])
jointsp2f = np.deg2rad([0,-17.0563224,30.31297739,-13.24660153,0,0])


#start target position
pos_inicial_pelves = [0., 0., 20.7]
pos_inicial_pe = [-(deslocamentoXpes/2), -1.7, -20.7]


#COMUNICACAO +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
'''spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 16000'''

#FUNÇÕES +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
'''Calcula cinematica inversa da pelves.   
	parm: indice(int) - diz em qual posicao do vetor de tragetoria deve ser armazenada a cinematica e qual momento da tragetoria calcular'''
def thread_cinematica_pelves(indice):
	pos = pos_inicial_pelves
	p = (deslocamentoXpes/2)*((np.exp((2*(indice-nEstados/2))/400) - np.exp((2*(indice-nEstados/2))/-400))/(np.exp((2*(indice-nEstados/2))/400)+np.exp((2*(indice-nEstados/2))/-400)))
	pos[0] = 0.45*p
	pos[1] = deslocamentoYpelves*np.sin(indice*np.pi/nEstados)
	frame_target = np.eye(4)
	frame_target[:3, 3] = pos
	ik = foot2pelv.inverse_kinematics(frame_target,initial_position=jointsf2p)
	ik = np.rad2deg(ik)
	ik = ik.astype(np.int8)
	#salva dados no array tragetoria
	#calc_lateral
	data_pelv[indice][0] = ik[0] 
	#calc_frontal
	data_pelv[indice][1] = ik[1] 
	#joelho
	data_pelv[indice][2] = ik[2] 
	#quadril
	data_pelv[indice][3] = ik[3] 
	#pelves
	data_pelv[indice][4] = ik[4]
	#torso 
	data_pelv[indice][5] = 0 
	#braco
	data_pelv[indice][6] = 0 	
	#cotovelo	
	data_pelv[indice][7] = 0


'''Calcula cinematica inversa dos pes.
	parm: indice(int) - diz em qual posicao do vetor de tragetoria deve ser armazenada a cinematica e qual momento da tragetoria calcular'''
def thread_cinematica_pe(indice):
	#calcula cinematica inversa
	pos = pos_inicial_pe
	#funcao tanh para suavizacao do deslocamento do pe
	pos[0] = (deslocamentoXpes/2)*((np.exp((2*(indice-nEstados/2))/400) - np.exp((2*(indice-nEstados/2))/-400))/(np.exp((2*(indice-nEstados/2))/400)+np.exp((2*(indice-nEstados/2))/-400)))
	pos[2] += deslocamentoZpes*np.exp(-((indice-nEstados/2)**2)/60000)
	frame_target = np.eye(4)
	frame_target[:3, 3] = pos
	ik = pelv2foot.inverse_kinematics(frame_target,initial_position=jointsp2f)
	ik = np.rad2deg(ik)
	ik = ik.astype(np.int8)
	#salva dados no array tragetoria
	#calc_lateral	
	data_foot[indice][0] = ik[4] 
	#calc_frontal
	data_foot[indice][1] = ik[3] 
	#joelho
	data_foot[indice][2] = ik[2] 
	#quadril
	data_foot[indice][3] = ik[1] 
	#pelves
	data_foot[indice][4] = ik[0]
	#torso 
	data_foot[indice][5] = 0 
	#braco
	data_foot[indice][6] = 0 	
	#cotovelo	
	data_foot[indice][7] = 0


'''Cria nEstados threads para calcular a cinematica inversa considerando que o intervalo de execucao T esta particionado em nEstados.'''
def calculaTragetoria():
	i = 0
	while i < nEstados:
		#cria threads para calcular cinematica invesa dos pes
		thread = threading.Thread(target=thread_cinematica_pe, args=(i, ))
		thread.daemon=True
		thread.start()
		thread.join()
		i += 1;
	i = 0
	while i < nEstados:
		#cria threads para calcular cinematica invesa da pelves
		thread = threading.Thread(target=thread_cinematica_pelves, args=(i, ))
		thread.daemon=True
		thread.start()
		thread.join()
		i += 1;


'''Plota linkagem.
	parm: corrente(chain) - conjunto de links que formam o manipulador
	parm: juntas(array) - vetor de rotacao das juntas
	parm: alvo(array) - posicao no espaco a ser marcada'''
def plot_chain (corrente, juntas=None, alvo=None):
	if juntas is None:
		juntas = [0] * len(corrente.links)
	ax = plot_utils.init_3d_figure()
	corrente.plot(juntas, ax, target=alvo)
	plot_utils.show_figure()


'''Calcula translacao para o eixo Y da pelves.
	parm: tempo(float) - qual instante do periodo de execucao do sinal a pelves esta'''
def calculaSinalY(tempo):
	Y = A*(1 + (l/g)*w*w)*np.sin(w*tempo)
	return Y


#SETUP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#print(np.rad2deg(ik))
#plot_chain(foot2pelv, juntas=jointsf2p)
#plot_chain(pelv2foot, juntas=jointsp2f, alvo=pos_inicial_pe)

#reading file
try:
	with open('data_foot.txt', 'r') as f:
		data_foot = np.loadtxt('data_foot.txt').reshape((nEstados,8))
		print ("File data_foot loaded!")	
	with open('data_pelv.txt', 'r') as f:
		data_pelv = np.loadtxt('data_pelv.txt').reshape((nEstados,8))
		print ("File data_pelv loaded!")
except IOError:
	print ("Calculando tragetoria.. ")
	calculaTragetoria()
	while threading.active_count() != 0:
		os.system("clear")
		print ("Calculando tragetoria.. (", threading.active_count(),"/",nEstados,")")
	np.savetxt('data_foot.txt', data_foot)
	np.savetxt('data_pelv.txt', data_pelv)

print(data_foot.shape)
print(data_pelv.shape)


#LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
		print ("fps:", fps)
		t_fps = 0
		fps = 0
	fps += 1
	
	#sending data
	#spi.writebytes(data[state].tolist())
	to_send = [254]+data_pelv[state].tolist()+[253]
	print (to_send)
	#time.sleep(0.01)

#END +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
f.close()


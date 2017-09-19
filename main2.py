import numpy as np
import os
import time
import ikpy as ik
import spidev
import serial

#outros parametros
freq = 2*np.pi/3.

#comunicacao
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 72000

#ser = serial.Serial('/dev/ttyACM0', 9600)

#perna esquerda
link0 = ik.link.URDFLink("calc_esq", [0, 0, 0], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True, bounds=(-60,60))
link1 = ik.link.URDFLink("pe_esq", [0, 0, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True, bounds=(-30,30))
link2 = ik.link.URDFLink("joelho_esq", [5, 0, 10] , [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True, bounds=(-60,60))
link3 = ik.link.URDFLink("coxa_esq", [-5, 0, 13], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True, bounds=(-80,80))
link4 = ik.link.URDFLink("quadril_esq", [0, 0, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
#perna direita
link5 = ik.link.URDFLink("pe_dir", [0, 3, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
link6 = ik.link.URDFLink("joelho_dir", [5, 0, 10] , [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link7 = ik.link.URDFLink("coxa_dir", [-5, 0, 13], [0, 0, 0], [0, 1, 0], use_symbolic_matrix=True)
link8 = ik.link.URDFLink("quadril_dir", [0, 0, 0], [0, 0, 0], [1, 0, 0], use_symbolic_matrix=True)
#centro de massa
link9 = ik.link.URDFLink("Pelv_esq", [0, 0, 23], [0, 0, 0], [0, 0, 1], use_symbolic_matrix=True)
#chains
perna_esq = ik.chain.Chain([link1, link0, link2, link3], [True, True, True, False])
perna_dir = ik.chain.Chain([link5, link6, link7, link8], [True, True, True, True])

joints = [0] * len(perna_esq.links)
joints2 = [0] * len(perna_dir.links)
target = [0, 0, 23]
frame_target = np.eye(4)
frame_target[:3, 3] = target
ik = perna_esq.inverse_kinematics(frame_target,initial_position=joints)
ik2 = perna_dir.inverse_kinematics(frame_target,initial_position=joints2)
ik = np.rad2deg(ik)
ik = ik.astype('int8')


#LOOP
start = time.time()
t = 0
t_fps = 0
fps = 0
fps2 = 0
t_inercial = 0
while 1:
        #timers
        dTime = time.time() - start
	start = time.time()
	t += dTime
	t_fps += dTime
	t_inercial += dTime

	#fps calculator
 	if t_fps > 1:
                '''os.system("clear")
                print "Tuplas/s:", fps
		print "\n"
		print "FPSext/s:", fps2
                '''
		t_fps = 0
                fps = 0
		fps2 = 0
	fps += 1        
        
        #sending data to execute
	to_send = np.array([254,0,1,2,3,4,5,6,7,253], dtype=np.uint8)
        spi.writebytes(to_send.tolist())
	
	#Inercial Sensor
	#if (t_inercial*1000) > 25:
		#t_inercial = 0
		#ser.write('#')
		#iner = np.array([0., 0., 0., 0.], dtype=np.float)
		#iner = ser.readline()
		#print type(iner), iner
		
	#STM (comunicacao)
	s = spi.readbytes(1)[0]
	if s == 0xFE:
		s = spi.readbytes(8)
		if int(spi.readbytes(1)[0]) == 0xFD:
			#pode usar dados do s
			fps2 += 1
			print "RECEBEU: ",s
	#else:
	#	print "Dado errado: ", s
	#print "ENVIOU:  ",to_send
        
	
	#time.sleep(0.01)

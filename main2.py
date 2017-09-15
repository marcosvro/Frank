import numpy as np
import os
import time
import ikpy as ik
import spidev
import tinyik

#outros parametros
freq = 2*np.pi/3.

#comunicacao
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 72000

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



arm = tinyik.Actuator(['y', [0,0,0],'x',[0,0,0],'y',[5,0,10],'y',[-5,0,13]])


#LOOP
start = time.time()
t = 0
t_fps = 0
fps = 0
while 1:
        #timers
        dTime = time.time() - start
	start = time.time()
	t += dTime
	t_fps += dTime

	#fps calculator
 	if t_fps > 1:
                os.system("clear")
                print "Tuplas/s:", fps
                t_fps = 0
                fps = 0
	#fps += 1        

	#solving inverse kinematic
        #variante = np.sin(t*freq) * 5
        #target[1] = variante
        #arm.ee = target
        #ik = np.round(np.rad2deg(arm.angles))
        
        #sending data
	to_send = np.array([254,0,1,2,3,4,5,6,7,253], dtype=np.uint8)
        spi.writebytes(to_send.tolist())
	
	#print spi.readbytes(10)
	if int(spi.readbytes(1)[0]) == 0xFE:
		s = spi.readbytes(8);
		if int(spi.readbytes(1)[0]) == 0xFD:
			#pode usar dados do s
			fps += 1
			#print "RECEBEU: ",s
        
	#print "ENVIOU:  ",to_send
        #time.sleep(0.01)
while 0:
        #timers
        dTime = time.time() - start
	start = time.time()
	t += dTime
	t_fps += dTime

	#fps calculator
 	if t_fps > 1:
                os.system("clear")
                print "fps:", fps
                t_fps = 0
                fps = 0
        fps += 1

	#solving inverse kinematic
        variante = np.sin(t*freq) * 5
        target[1] = variante
        frame_target[:3, 3] = target
        ik = perna_esq.inverse_kinematics(frame_target,initial_position=joints)
        ik = np.rad2deg(ik)
        ik = ik.astype('int8')

        #sending to slave
        to_send = np.array([127,0,1,2,3,4,5,6,7,8,9,10,11, -127], dtype=np.int8)
        #to_send[0] = ik[0]
        #to_send[1] = ik[1]
        #to_send[2] = ik[2]
        spi.xfer(to_send.tolist())
        #print to_send
        #time.sleep(0.01)

#from picamera.array import PiRGBArray
#from picamera import PiCamera
#from math import tan
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

import cruzamento
import controleMotores as motor
import obstaculo
import resgate
import verde
import ultrassonico as us

#~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#import ultrasonic_distance.py as u1
#import ultrasonic_distance.py as u2
#distancia = u1.distance()
#~~~~~~~~~~~~~~~~~~~~~~~~~~~#

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT)
GPIO.output(40, GPIO.HIGH) #pino da luz da câmera - acende

camera = cv2.VideoCapture(0)
hcamera = 200
wcamera = 125
camera.set(3, hcamera)  #resolução
camera.set(4, wcamera)
setpoint = 80 #valor equivalente a (largura em pixel da câmera)/2
# é bom lembrar que o tamanho da imagem e resolução são valores distintos

time.sleep(0.1)

kernel = np.ones((3,3), np.uint8)
x_last = 100
y_last = 80


botao = 3 #pin

motor.motorSetup()
velocidadePadrao = 82

start_time = time.time()  #contadores para encontrar o fps
counter = 0

while(True):

#	if botao == HIGH : # rampa
#		velocidadePadrao = 140
#		lock = True
#	if lock == False :
#		obstaculo.obstaculo()

	ret, image = camera.read()
	counter +=1
	interesse = image[119:120,0:160]

	Blackline = cv2.inRange(image[60:120,0:160], (0,0,0), (75,75,75)) # 0,0,0 à 75,75,75 (preto)
	Pontoverde = cv2.inRange(interesse, (0,100,0), (100,200,100)) # 0,65,0 à 100,200,100 (verde)

	kernel = np.ones((3,3), np.uint8)
	Blackline = cv2.erode(Blackline, kernel, iterations=1)
	Pontoverde = cv2.erode(Pontoverde, kernel, iterations=1)

	contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours_ver, hierarchy_ver = cv2.findContours(Pontoverde.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours_blk_len = len(contours_blk)
	num_contornos_verde = len(contours_ver)

#	temCruzamento() retorna ou False ou uma lista com dois valores (os pontos centrais do cruzamento)
	cruz = cruzamento.temCruzamento(Blackline[0:110,0:160])
	
	if cruz != False and num_contornos_verde > 0 :
		Pontoverde = cv2.inRange(image, (0,100,0), (100,200,100))
		contours_ver, hierarchy_ver = cv2.findContours(Pontoverde.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		num_contornos_verde = len(contours_ver)
		verde.Pontos_Verdes(contours_ver, num_contornos_verde, cruz)


	if contours_blk_len > 0 : #se existem contornos
		if contours_blk_len == 1 : #se existe um contorno
			blackbox = cv2.minAreaRect(contours_blk[0]) #cria-se o menor retângulo com o contorno dentro
#			x_pre, y_pre, w_pre, h_pre = cv2.boundingRect(contours_blk[0]) #para interação com a linha verde
	
		else: #se existe 2+ contornos
			candidatos=[] #array vazio
			off_bottom = 0	   
			for con_num in range(contours_blk_len):	#para cada contorno na imagem
				blackbox = cv2.minAreaRect(contours_blk[con_num]) #menor ret para o respectivo contorno
				(x_min, y_min), (w_min, h_min), ang = blackbox
				box = cv2.boxPoints(blackbox) #canto mais abaixo do retângulo
				(x_box,y_box) = box[0]
				if y_box > (hcamera-2) : #se encosta na parte de baixo da imagem
					off_bottom += 1
				candidatos.append((y_box,con_num,x_min,y_min))
			candidatos = sorted(candidatos) #arruma os candidatos em ordem crescente ***
			if off_bottom > 1:
				candidatos_abaixo_da_tela=[]
				for con_num in range((contours_blk_len - off_bottom), contours_blk_len): # *** para tirar os últimos num (os maiores)
					(y_highest,con_highest,x_min, y_min) = candidatos[con_num]
					total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5 #pitágoras  menor distância entre o contorno anterior e o novo
					candidatos_abaixo_da_tela.append((total_distance,con_highest)) #acrescenta ao array a distância e o num do contorno
				candidatos_abaixo_da_tela = sorted(candidatos_abaixo_da_tela) #organiza em ordem crescente
				(total_distance,con_highest) = candidatos_abaixo_da_tela[0] #os menores núm vão ser os com menor distância do contorno anterior, portanto a linha certa (o espaço 0 em ordem crescente) 
				blackbox = cv2.minAreaRect(contours_blk[con_highest]) #cria-se o menor retângulo do contorno certo
			else: #se tem mais q um contorno na tela, mas apenas um (ou nenhum) tocando embaixo na img
				(y_highest,con_highest,x_min, y_min) = candidatos[contours_blk_len-1] #-1 por que o Array começa em 0 (ex: o 3º contorno é o [2])
				blackbox = cv2.minAreaRect(contours_blk[con_highest]) #cria-se o menor retângulo do contorno certo
	
		(x_min, y_min), (w_min, h_min), ang = blackbox #extrai info da blackbox
		x_last = x_min #guarda info da blackbox para uso no próx frame
		y_last = y_min
		if ang < -45 : #se a blackbox tombar para a direita   CORREÇÃO DE VALOR ANGULARES
			ang = 90 + ang #o ângulo negativo é transformado em positivo (mais alinhado na horizontal, menor valor)
		if w_min < h_min and ang > 0: #se a largura da blackbox for menor q a altura e ang maior q 0
			ang = (90-ang)*-1 
		if w_min > h_min and ang < 0: #se a largura da blackbox for maior q a altura e ang menor q 0
			ang = 90 + ang	  
		
		error = int(x_min - setpoint) #distância da linha ao centro do frame
		ang = int(ang)	 #ang da linha
		motor.direcaoMotor (velocidadePadrao, (error*kp)+(ang*ap))

		box = cv2.boxPoints(blackbox) #para baixo são comandos para desenhar na tela (desnecessário)
		box = np.int0(box)
		cv2.drawContours(image,[box],0,(0,0,255),3)	 
		cv2.putText(image,str(ang),(10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
		cv2.putText(image,str(error),(10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#		cv2.putText(image,str(off_bottom),(10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
		cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
	
		
	cv2.imshow("original with line", image)
	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):       # Apertando "q" o programa é encerrado
		break
finish_time = time.time()
fps = counter / (finish_time - start_time)
print ("frames per second = " + str(fps))
GPIO.output(40, GPIO.LOW)  #pino da luz da câmera - apaga
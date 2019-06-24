#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import cv2
import numpy as np
#import RPi.GPIO as GPIO

#~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#import ultrasonic_distance.py as u1
#import ultrasonic_distance.py as u2
#distancia = u1.distance()
#~~~~~~~~~~~~~~~~~~~~~~~~~~~#

#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(40, GPIO.OUT)
#GPIO.output(40, GPIO.HIGH) #pino da luz da câmera - acende

#camera = PiCamera()
#camera.resolution = (200, 125)  #resolução baixa para menor fps
#camera.rotation = 180
#camera.framerate = 60  #taxa máx de fps
#rawCapture = PiRGBArray(camera, size=(200, 120))
camera = cv2.VideoCapture(0)
hcamera = 200
wcamera = 125
camera.set(3, hcamera)  #resolução
camera.set(4, wcamera)
setpoint = 80 #valor equivalente a (largura em pixel da câmera)/2

time.sleep(0.1)

kernel = np.ones((3,3), np.uint8)
x_last = 100   #X e Y utilizados para encontrar a linha certa (qndo tem +q 1)
y_last = 80

start_time = time.time()  #contadores para encontrar o fps
counter = 0

#def obstaculo():
#	distanciaS1 = u1.distance()  #sensor da frente (S1) esquerda (S2)
#	if distanciaS1 < 5 :
#		distanciaS2 = u2.distance()
#		while(distanciaS1 < distanciaS2):
#			distanciaS2 = u2.distance()
#			Motor_Steer(BP.PORT_A, BP.PORT_D, 50, 200) #dando volta no eixo
#		Motor_Steer(BP.PORT_A, BP.PORT_D, 50, 20)
#		time.sleep(1.2)
#		Motor_Steer(BP.PORT_A, BP.PORT_D, 50, -20)
#		time.sleep(0.2)

def temCruzamento(imgBinaria):
	yArray=[]
	xArray=[]
	hImg, wImg = imgBinaria.shape
	for y in range(0,hImg) :
		for x in range(0,wImg) :
			if imgBinaria[y,x] == 255 :
				if x == wImg-1 :
					yArray.append(y)
			else:
				break
	for x in range(0,wImg) :
		for y in range(0,hImg) :
			if imgBinaria[y,x] == 255 :
				if y == hImg-1 :
					xArray.append(x)
			else:
				break

	if len(yArray) >= 2 and len(xArray) < 2 :
		for x in range(0,wImg) :
			for y in range(0,yArray[0]) :
				if imgBinaria[y,x] == 255 :
					if y == yArray[0]-1 :
						xArray.append(x)
				else:
					break
			for y in range(yArray[-1],hImg) :
				if imgBinaria[y,x] == 255 :
					if y == hImg-1 :
						xArray.append(x)
				else:
					break
	elif len(xArray) >= 2 and len(yArray) < 2 :
		for y in range(0,hImg) :
			for x in range(0,xArray[0]) :
				if imgBinaria[y,x] == 255 :
					if x == xArray[0]-1 :
						yArray.append(y)
				else:
					break
			for x in range(xArray[-1],wImg) :
				if imgBinaria[y,x] == 255 :
					if x == wImg-1 :
						yArray.append(y)
				else:
					break
	if len(yArray) >= 2 and len(xArray) >= 2 :
		y = (yArray[0]+((yArray[-1]-yArray[0])/2))
		x = (xArray[0]+((xArray[-1]-xArray[0])/2))
		return (y,x)
	else:
		return False

def Pontos_Verdes (contornos_verdes, num_contornos, cruzamento):
	yCruz, xCruz = cruzamento
	yCentro=[]
	if num_contornos == 1 :
		x_ver, y_ver, w_ver, h_ver = cv2.boundingRect(contornos_verdes[0])
		xCentro = (x_ver+((w_ver-x_ver)/2))
		if xCruz > xCentro :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 45) #esquerda
			time.sleep(0.3)
		elif xCruz < (x_ver+((w_ver-x_ver)/2)) :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, -45) #direita
			time.sleep(0.3)

	elif num_contornos == 2 :
		x_ver[0], y_ver[0], w_ver[0], h_ver[0] = cv2.boundingRect(contornos_verdes[0])
		x_ver[1], y_ver[1], w_ver[1], h_ver[1] = cv2.boundingRect(contornos_verdes[1])
		yCentro[0] = (y_ver[0]+((y_ver[0]-y_ver[0])/2))
		yCentro[1] = (y_ver[1]+((y_ver[1]-y_ver[1])/2))
		xCentro = (x_ver[1]+((w_ver[1]-x_ver[1])/2))
		if yCruz < yCentro[0] and yCruz < yCentro[1] : #se os dois pontos verdes estão em baixo
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 200) #200 pq a velocidade dos motores ficam speed e -speed
			time.sleep(1)
		elif yCruz > yCentro[0] and yCruz > yCentro[1] :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 0) #reto
			time.sleep(0.5)
		elif xCruz > xCentro :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 45) #esquerda
			time.sleep(0.3)
		elif xCruz < xCentro :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, -45) #direita
			time.sleep(0.3)

	elif num_contornos == 3 :
		x_ver[0], y_ver[0], w_ver[0], h_ver[0] = cv2.boundingRect(contornos_verdes[0])
		x_ver[1], y_ver[1], w_ver[1], h_ver[1] = cv2.boundingRect(contornos_verdes[1])
		x_ver[2], y_ver[2], w_ver[2], h_ver[2] = cv2.boundingRect(contornos_verdes[2])
		yCentro[0] = (y_ver[1]+((y_ver[1]-y_ver[1])/2))
		yCentro[1] = (y_ver[2]+((y_ver[2]-y_ver[2])/2))
		xCentro = (x_ver[2]+((w_ver[2]-x_ver[2])/2))
		if yCruz < yCentro[0] and yCruz < yCentro[1]:
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 200) #200 pq a velocidade dos motores ficam speed e -speed
			time.sleep(1)
		elif xCruz > xCentro :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 45) #esquerda
			time.sleep(0.3)
		elif xCruz < xCentro :
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, -45) #direita
			time.sleep(0.3) 

	elif num_contornos == 4 :
#		Motor_Steer (BP.PORT_A, BP.PORT_D, 82, 200) #200 pq a velocidade dos motores ficam speed e -speed
		time.sleep(1)

	else:
		pass

def Motor_Steer (port1, port2, speed, steering): #vira com base no ângulo em relação a linha preta | função de controle dos motores do carro
	if steering == 0: #caso alinhado com a linha preta, reto
#		BP.set_motor_speed(port1, speed)
#		BP.set_motor_speed(port2, speed)
		return
	elif steering > 0: #vira esquerda
		steering = 100 - steering
#		BP.set_motor_speed(port2, speed)
#		BP.set_motor_speed(port1, speed*steering/100)
		return
	elif steering < 0: #vira direita
		steering = steering * -1
		steering + 100 - steering
#		BP.set_motor_speed(port1, speed)
#		BP.set_motor_speed(port2, speed*steering/100)
		return
		  #variáveis configuráveis para definir a direção do carro considerando a posição da linha
kp = .75  #com base na distância da linha e do centro do frame
kp2 = 1.25
ap = 1    #com base no âng da linha

while(True):
	ret, image = camera.read()
	Encontrouverde = False
	counter +=1
	interesse = image[119:120,0:160]

	Blackline = cv2.inRange(image[60:120,0:160], (0,0,0), (75,75,75))	#todas as cores de 0,0,0 à 75,75,75 (preto) definem a linha preta
	Pontoverde = cv2.inRange(interesse, (0,100,0), (100,200,100)) #todas as cores de 0,65,0 à 100,200,100 (verde) definem as curvas verdes
	#cv2.imshow("Binario Linha Preta", Blackline)  #comando interessante para teste

	kernel = np.ones((3,3), np.uint8) #n sei
	Blackline = cv2.erode(Blackline, kernel, iterations=1) #espreme a linha, apenas maiores áreas têm contorno
#	Blackline = cv2.dilate(Blackline, kernel, iterations=9)
	Pontoverde = cv2.erode(Pontoverde, kernel, iterations=1) #espreme a cor verde, apenas áreas maiores têm contorno
#	Pontoverde = cv2.dilate(Pontoverde, kernel, iterations=3)
#	cv2.imshow("a", Pontoverde)

	contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #encontra o contorno da linha preta
	contours_ver, hierarchy_ver = cv2.findContours(Pontoverde.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #encontra o contorno do ponto verde
	contours_blk_len = len(contours_blk) #num de contornos pretos
	num_contornos_verde = len(contours_ver) #num de contornos verdes

	cruzamento = temCruzamento(Blackline[0:110,0:160])
	obstaculo()
	
	if cruzamento != False :
		Pontoverde = cv2.inRange(image, (0,100,0), (100,200,100))
		contours_ver, hierarchy_ver = cv2.findContours(Pontoverde.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		num_contornos_verde = len(contours_ver)
		Pontos_Verdes(contours_ver, num_contornos_verde, cruzamento)

#			OBJETIVO: testar se encontrou algum verde e este está na linha inferior da tela
#						usa o ponto médio do contorno pra definir a direção	
#			greenbox = cv2.minAreaRect(contours_ver[0])
#			x_ver, y_ver, w_ver, h_ver = cv2.boundingRect(contours_ver[0]) #cria um retângulo em torno da área verde e extrai info dele
#			(x_min, y_min), (w_min, h_min), ang = greenbox 
#			centerx_ver = x_ver + (w_ver/2) #largura/2
#			error = int(x_min - setpoint)
#			if ang < -45 : #se a greenbox tombar para a direita   CORREÇÃO DE VALOR ANGULARES
#				ang = 90 + ang #o ângulo negativo é transformado em positivo (mais alinhado na horizontal, menor valor)
#			if w_min < h_min and ang > 0: #se a largura da greenbox for menor q a altura e ang maior q 0
#				ang = (90-ang)*-1 
#			if w_min > h_min and ang < 0: #se a largura da greenbox for maior q a altura e ang menor q 0
#				ang = 90 + ang
#			cv2.line(image, (int(centerx_ver), 200), (int(centerx_ver), 250),(0,0,255),3) #cria uma linha
#			Motor_Steer (BP.PORT_A, BP.PORT_D, 82, (error*kp2)+(ang*ap))



	if contours_blk_len > 0 : #se existem contornos
		if contours_blk_len == 1 : #se existe um contorno
			blackbox = cv2.minAreaRect(contours_blk[0]) #cria-se o menor retângulo com o contorno dentro
#			x_pre, y_pre, w_pre, h_pre = cv2.boundingRect(contours_blk[0]) #para interação com a linha verde
	 
		else: #se existe 2+ contornos
			canditates=[] #array vazio
			off_bottom = 0	   
			for con_num in range(contours_blk_len):	#para cada contorno na imagem
				blackbox = cv2.minAreaRect(contours_blk[con_num]) #menor ret para o respectivo contorno
				(x_min, y_min), (w_min, h_min), ang = blackbox #pega info da blackbox
				box = cv2.boxPoints(blackbox) #canto mais abaixo do retângulo
				(x_box,y_box) = box[0] #cordenadas do ponto
				if y_box > (hcamera-2) : #se encosta na parte de baixo da imagem
					off_bottom += 1 #conta mais um
				canditates.append((y_box,con_num,x_min,y_min)) #acrescenta ao Array
			canditates = sorted(canditates) #arruma os candidatos em ordem crescente ***
			if off_bottom > 1: #se tem mais do q uma linha tocando embaixo na img 
				canditates_off_bottom=[]
				for con_num in range((contours_blk_len - off_bottom), contours_blk_len): # *** para tirar os últimos num (os maiores)
					(y_highest,con_highest,x_min, y_min) = canditates[con_num] #pega info do respectivo contorno | con_num em canditates[] vira com_highest
					total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5 #usa-se pitágoras para encontrar a menor distância entre o contorno anterior e o novo | **0.5 seria raiz quadrada (elevado à fração)
					canditates_off_bottom.append((total_distance,con_highest)) #acrescenta ao array a distância e o num do contorno
				canditates_off_bottom = sorted(canditates_off_bottom) #organiza em ordem crescente
				(total_distance,con_highest) = canditates_off_bottom[0] #os menores núm vão ser os com menor distância do contorno anterior, portanto a linha certa (o espaço 0 em ordem crescente) 
				blackbox = cv2.minAreaRect(contours_blk[con_highest]) #cria-se o menor retângulo do contorno certo
			else: #se tem mais q um contorno na tela, mas apenas um (ou nenhum) tocando embaixo na img
				(y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1] #-1 por que o Array começa em 0 (ex: o 3º contorno é o [2])
				blackbox = cv2.minAreaRect(contours_blk[con_highest]) #cria-se o menor retângulo do contorno certo
#			x_pre, y_pre, w_pre, h_pre = cv2.boundingRect(contours_blk[con_highest]) #para interação com a linha verde
	   
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
#		Motor_Steer (BP.PORT_A, BP.PORT_D, 82, (error*kp)+(ang*ap)) #82 é a velocidade, |ainda configurar|
		box = cv2.boxPoints(blackbox) #para baixo são comandos para desenhar na tela (desnecessário)
		box = np.int0(box)
		cv2.drawContours(image,[box],0,(0,0,255),3)	 
		cv2.putText(image,str(ang),(10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
		cv2.putText(image,str(error),(10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#		cv2.putText(image,str(off_bottom),(10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
		cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
	 
		  
	cv2.imshow("original with line", image)
#	rawCapture.truncate(0)
	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):       # Apertando "q" o programa é encerrado
		break
finish_time = time.time()
fps = counter / (finish_time - start_time)
print ("frames per second = " + str(fps))
#GPIO.output(40, GPIO.LOW)  #pino da luz da câmera - apaga
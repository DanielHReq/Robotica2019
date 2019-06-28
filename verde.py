import time
import cv2
import ControleMotores as motor

def Pontos_Verdes (contornos_verdes, num_contornos, cruzamento):
	yCruz, xCruz = cruzamento
	yCentro=[]
	if num_contornos == 1 :
		x_ver, y_ver, w_ver, h_ver = cv2.boundingRect(contornos_verdes[0])
		xCentro = (x_ver+((w_ver-x_ver)/2))
		if xCruz > xCentro :
			motor.esquerda()
			time.sleep(0.3)
		elif xCruz < (x_ver+((w_ver-x_ver)/2)) :
			motor.direita()
			time.sleep(0.3)

	elif num_contornos == 2 :
		x_ver[0], y_ver[0], w_ver[0], h_ver[0] = cv2.boundingRect(contornos_verdes[0])
		x_ver[1], y_ver[1], w_ver[1], h_ver[1] = cv2.boundingRect(contornos_verdes[1])
		yCentro[0] = (y_ver[0]+((y_ver[0]-y_ver[0])/2))
		yCentro[1] = (y_ver[1]+((y_ver[1]-y_ver[1])/2))
		xCentro = (x_ver[1]+((w_ver[1]-x_ver[1])/2))
		if yCruz < yCentro[0] and yCruz < yCentro[1] : #se os dois pontos verdes estão em baixo
			motor.centro_dir ()
			time.sleep(1)
		elif yCruz > yCentro[0] and yCruz > yCentro[1] :
			motor.reto ()
			time.sleep(0.5)
		elif xCruz > xCentro :
			motor.esquerda ()
			time.sleep(0.3)
		elif xCruz < xCentro :
			motor.direita ()
			time.sleep(0.3)

	elif num_contornos == 3 :
		x_ver[0], y_ver[0], w_ver[0], h_ver[0] = cv2.boundingRect(contornos_verdes[0])
		x_ver[1], y_ver[1], w_ver[1], h_ver[1] = cv2.boundingRect(contornos_verdes[1])
		x_ver[2], y_ver[2], w_ver[2], h_ver[2] = cv2.boundingRect(contornos_verdes[2])
		yCentro[0] = (y_ver[1]+((y_ver[1]-y_ver[1])/2))
		yCentro[1] = (y_ver[2]+((y_ver[2]-y_ver[2])/2))
		xCentro = (x_ver[2]+((w_ver[2]-x_ver[2])/2))
		if yCruz < yCentro[0] and yCruz < yCentro[1]:
			motor.centro_dir ()
			time.sleep(1)
		elif xCruz > xCentro :
			motor.esquerda (82, 45)
			time.sleep(0.3)
		elif xCruz < xCentro :
			motor.direita ()
			time.sleep(0.3) 

	elif num_contornos == 4 :
		motor.centro_dir (82, 200)
		time.sleep(1)

	else:
		pass
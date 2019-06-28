dir = 10 #pin
esq = 11

def motorSetup():
	BP.set_motor_speed(dir, 0)
	BP.set_motor_speed(esq, 0)
	return

def direcaoMotor (speed, steering): #vira com base no ângulo em relação a linha preta | função de controle dos motores do carro
	if steering == 0: #caso alinhado com a linha preta, reto
#		BP.set_motor_speed(dir, speed)
#		BP.set_motor_speed(esq, speed)
		print("reto")
		return
	elif steering > 0: #vira esquerda
		steering = 100 - steering
#		BP.set_motor_speed(esq, speed)
#		BP.set_motor_speed(dir, speed*steering/100)
		if(steering>70):
			print("esquerda suave")
		else:
			print("esquerda")
		return
	elif steering < 0: #vira direita
		steering = steering * -1
		steering = 100 - steering
#		BP.set_motor_speed(dir, speed)
#		BP.set_motor_speed(esq, speed*steering/100)
		if(steering>70):
			print("direita suave")
		else:
			print("direita")
		return

def esquerda ():
	BP.set_motor_speed(esq, 82)
	BP.set_motor_speed(dir, 82*55/100)
	return

def direita ():
	BP.set_motor_speed(esq, 82*55/100)
	BP.set_motor_speed(dir, 82)
	return

def reto ():
	BP.set_motor_speed(esq, 82)
	BP.set_motor_speed(dir, 82)
	return

def tras ():
	BP.set_motor_speed(esq, -50)
	BP.set_motor_speed(dir, -50)
	return

def centro_dir ():
	BP.set_motor_speed(esq, 50)
	BP.set_motor_speed(dir, -50)
	return

def centro_esq ():
	BP.set_motor_speed(esq, -50)
	BP.set_motor_speed(dir, 50)
	return
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
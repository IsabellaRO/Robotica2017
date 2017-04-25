# minhas funcoes

from random import randint, gauss
import math
import inspercles
reload(inspercles)

def criaParticulas(n, minx, maxx, miny, maxy):
    L=[]
    for i in range(n):
        p = []
        p.append(randint(minx,maxx+1))
        p.append(randint(miny,maxy+1))
        p.append(randint(0, 361))
        L.append(p)
    return L

def moveParticulas(L, delta_x, delta_y, delta_theta, std_x, std_y, std_theta):
	for p in L:
		newdeltax = delta_x + gauss(0, std_x)
		newdeltay = delta_y + gauss(0, std_y)
		newdeltat = delta_theta + gauss(0, std_theta)
		p.x = p.x+newdeltax
		p.y = p.y+newdeltay
		p.theta = p.theta+newdeltat
	return L


def calculaPDH(L, pose, angles, np_image):
	leituras, lidar_map = inspercles.nb_simulate_lidar(pose, angles, np_image)
	PDH = []
	for p in L:
		poseP = [p.x, p.y, p.theta]
		leiturasP, lidar_mapP = inspercles.nb_simulate_lidar(poseP, angles, np_image)
		x = 0
		for i in range(len(angles)):
			x += math.exp((-(leituras[angles[i]]-leiturasP[angles[i]]))/(2*5**2))
		PDH.append(x)

	alpha = 1/sum(PDH)
	for i in range(len(PDH)):
		PDH[i] *= alpha

	return PDH


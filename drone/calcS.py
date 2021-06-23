import numpy as np
import math

def calcS(psi):
    phi = psi[0,0]
    theta = psi[1,0]

    S = np.matrix([[math.cos(theta), 0., -math.cos(phi)*math.sin(theta)], [0., 1., math.sin(theta)], [math.sin(theta), 0., math.cos(phi)*math.cos(theta)]])
    return S

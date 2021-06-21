import numpy as np
import math


def calcRy(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    Ry = np.matrix([[c,0,s], [0,1,0], [-s,0,c]])
    return Ry

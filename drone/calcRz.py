import numpy as np
import math


def calcRz(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Rz

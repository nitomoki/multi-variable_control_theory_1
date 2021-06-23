import numpy as np
import math


def calcRx(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    Rx = np.matrix([[1.,0.,0.], [0.,c,-s], [0.,s,c]])
    return Rx

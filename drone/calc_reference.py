import numpy as np
import math

def calc_reference(t):
    p_d  = np.matrix([0.5*math.cos(0.5*t), 0.5*math.sin(0.5*t), 1.]).T
    psi_d = 0.

    return (p_d, psi_d)


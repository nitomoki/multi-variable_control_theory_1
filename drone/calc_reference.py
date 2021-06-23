import numpy as np
import math

def calc_reference(t, shape='circle'):
    if shape=='circle':
        p_d  = np.matrix([0.5*math.cos(0.5*t), 0.5*math.sin(0.5*t), 1.]).T
        psi_d = 0.

    if shape=='spiral':
        p_d  = np.matrix([0.5*math.cos(t), 0.5*math.sin(t), 0.2*t]).T
        psi_d = 0.

    if shape=='eight':
        p_d  = np.matrix([0.5*math.sin(t), 0.5*math.sin(2*t), 1.]).T
        psi_d = 0.

    return (p_d, psi_d)


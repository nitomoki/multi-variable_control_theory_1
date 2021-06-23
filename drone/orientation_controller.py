import numpy as np
import math
from icecream import ic

from GLOBAL_VARIABLE import *
from calcS import *


K_d = np.matrix(np.diag([1.,1.,1.]))* 50.
K_s = np.matrix(np.diag([1.,1.,1.]))* 300.

def orientation_controller(x, Phi_d):
    Phi = x[2,:].T
    Omega = x[3,:].T
    S = calcS(Phi)

    Omegadot = -K_d*Omega + np.linalg.inv(S).T*K_s*(Phi_d - Phi)
    u_rot = I*Omegadot + np.cross(Omega.T, (I*Omega).T).T

    return u_rot

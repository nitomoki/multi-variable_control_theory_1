import numpy as np
import math
from icecream import ic

from GLOBAL_VARIABLE import *
from calcR import *
from calcS import *

def drone(x,u):
    v = x[1,:].T
    Psi = x[2,:].T
    Omega = x[3,:].T

    R = calcR(Psi)
    S = calcS(Psi)

    pdot = v
    vdot = np.matrix([0., 0., -g]).T + R*np.matrix([0., 0., u[0,0]/m]).T
    psidot = np.linalg.inv(S)*Omega
    omegadot = np.linalg.inv(I)*(u[1:4] - np.matrix(np.cross(Omega.T, (I*Omega).T)).T)

    xd = np.concatenate((pdot.T, vdot.T, psidot.T, omegadot.T))

    return xd





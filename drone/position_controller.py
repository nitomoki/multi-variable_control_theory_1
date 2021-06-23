import numpy as np
import math
from icecream import ic

from GLOBAL_VARIABLE import *

K = np.matrix(np.diag([1.,1.,1.])) * 25.
C = np.matrix(np.diag([1.,1.,1.])) * 10.

def position_controller(x, p_d):

    u = np.matrix([0., 0., m*g]).T + K*(p_d - x[0].T) - C*x[1].T
    u_z = np.linalg.norm(u)
    (phi_d, theta_d) = _calc_p_t(u)

    return (u, u_z, phi_d, theta_d)

def _calc_p_t(v):
    x = v[0,0]
    y = v[1,0]
    z = v[2,0]
    theta = math.atan(-x/z)
    ct = math.cos(theta)
    st = math.sin(theta)
    phi = math.atan(-y/(st*x - ct*z))
    #cp = math.cos(phi)
    #sp = math.sin(phi)
    #Rphi = np.matrix([[1.,0.,0.], [0., cp, -sp], [0., sp, cp]])
    #Rtheta = np.matrix([[ct, 0., st], [0., 1., 0.], [-st, 0., ct]])
    return (-phi, -theta)

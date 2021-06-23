import numpy as np

from calcRx import *
from calcRy import *
from calcRz import *

def calcR(psi):
    R = calcRz(psi[2,0])*calcRx(psi[0, 0])*calcRy(psi[1,0])
    return R

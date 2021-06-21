import numpy as np

from calcRx import *
from calcRz import *
from calcRz import *

def calcR(psi):
    R = calcRz(psi[2])*calcRx(psi[0])*calcRy(psi[1])
    return R

import math

def wrap_pi(x):
    y = x
    while y>math.pi:
        y = y-2*math.pi
    while y<-math.pi:
        y = y+2*math.pi
    return y

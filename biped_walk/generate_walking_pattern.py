from icecream import ic
import numpy as np
import math

def generate_walking_pattern(step_duration, T, stride=0.3, spacing=0.2):
    N = 10
    #stride  = 0.3
    #spacing = 0.2
    footstep = np.zeros((8,N))
        # generate footstep
        # columns: k
        # rows   : pos_x pos_y pos_z theta dcm_x dcm_y zmp_x zmp_y
    footstep[0:4, 0] = np.array([0, -spacing/2, 0, 0]) # 1st step right
    footstep[0:4, 1] = np.array([0,  spacing/2, 0, 0]) # 2nd step left
    

    # intermediate footsteps
    sign = -1
    for k in range(1, N-2):
        footstep[0:4, k+1] = footstep[0:4, k] + np.array([stride/2, sign*spacing, 0, 0])
        sign = -sign
    # last footstep
    footstep[0:4,N-1] = footstep[0:4, N-2] + np.array([0, sign*spacing, 0, 0])

    # generate reference zmp
    for k in range(0, N-1):
        # set the same as foot position
        footstep[6:8, k] = footstep[0:2, k]
    # last reference zmp is set to the middle of two feets
    footstep[6:8, N-1] = (footstep[0:2, N-1] + footstep[0:2, N-2])/2

    # generate reference dcm
    # last dcm is set to the middle of footsteps
    footstep[4:6, N-1] = (footstep[0:2, N-1] + footstep[0:2, N-2])/2
    footstep[4:6, N-2] = (footstep[0:2, N-1] + footstep[0:2, N-2])/2

    for k in range(N-1, 1, -1):
        footstep[4:6, k-1] = footstep[6:8, k-1] + math.exp(-step_duration/T)*(footstep[4:6, k] - footstep[6:8, k-1])

    # initial dcp is set to the middole of footsteps
    footstep[4:6, 0] = (footstep[0:2, 0] + footstep[0:2, 1])/2

    # adjust initial zmp to make it consistent with initial dcm
    footstep[6:8, 0] = (footstep[4:6, 0] - math.exp(step_duration/T)*footstep[4:6, 0])/(1 - math.exp(step_duration/T))

    return footstep

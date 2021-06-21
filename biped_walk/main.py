import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as animation
import numpy as np
from control import *
import math
from icecream import ic
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import seaborn as sns
from scipy.stats import multivariate_normal
from generate_walking_pattern import *
from generate_noise import *
from wrap_pi import *


sns.set()

playing = True
#playing = False
# parameters
h = 1.0
g = 9.8
step_duration = 0.5
foot_size = np.array([0.2, 0.2])
f_size_len = 0.5 * math.sqrt(foot_size[0]**2 + foot_size[1]**2)
f_theta_rad = math.acos(foot_size[0]*0.5/f_size_len)
swing_height = 0.03
# constatnt of LIPM
T = math.sqrt(h/g)
# time step
dt = 0.01
# animation_speed
animation_speed = 1.0
# generate walking pattern
footstep = generate_walking_pattern(step_duration, T)
# number of footstep
N = footstep.shape[1]

# liner system
A = np.matrix([[0.0, 1.0, 0.0, 0.0],\
               [1.0/T**2, 0.0, 0.0, 0.0],\
               [0.0, 0.0, 0.0, 1.0],\
               [0.0, 0.0, 1.0/T**2, 0.0]])
B = np.matrix([[0.0, 0.0],\
               [-1/T**2, 0.0],\
               [0.0, 0.0],\
               [0.0, -1/T**2]])

# design feedback gain
Q = np.diag([1.0, 1.0, 1.0, 1.0])
R = np.diag([100000.0, 100000.0])
K, P, e = lqr(A, B, Q, R)

ic(Q)
ic(R)
ic(K)


fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')

# variables
class State:
    def __init__(self):
        self.sup = 0 # current support foot 0:right 1:left
        self.swg = 1 # current swing foot   0:right 1:left
        self.k = 1 # current footstep index
        self.t = 0 # elapsed time since last foot exchange
        self.t_total = 0 # total elapsed time
        self.foot = np.matrix([[0.0, 0.0],\
                               [0.0, 0.0],\
                               [0.0, 0.0],\
                               [0.0, 0.0]])
                             # foot pose
                             # rows: x y z theta
                             # columns: right left
        self.p_com = np.matrix([0.0, 0.0])
        self.v_com = np.matrix([0.0, 0.0])
        self.a_com = np.matrix([0.0, 0.0])
        self.p_zmp = np.matrix([0.0, 0.0])
        self.p_com_ref = np.matrix([0.0, 0.0])
        self.p_dcm_ref = footstep[4:6, self.k]
        self.p_zmp_ref = footstep[6:8, self.k]
        self.x = np.matrix([self.p_com[0, 0], self.v_com[0, 0], self.p_com[0, 1], self.v_com[0, 1]])
        self.u = self.p_zmp



def main():
    state = State()
    if playing:
        anim = animation.FuncAnimation(fig, update, fargs=(state,), frames=range(256), interval=100)
        plt.show()

def update(frame, state):
    ax.cla()
    ax_config()
    simulation(state)
    com = [state.x[0, 0], state.x[0, 2], h]
    zmp = [state.u[0, 0], state.u[0, 1], 0.0]
    draw_com(com)
    draw_zmp(zmp)
    draw_line(com, zmp)
    draw_foot_r(state.foot)
    draw_foot_l(state.foot)
    #ic.disable()

def simulation(state):

    state.foot[:, state.sup] = np.matrix(footstep[0:4, state.k]).transpose()

    if state.k < N-1:
        # swing foot pose is expressed as a cycloid curve connecting two footstep
        s = state.t/step_duration
        # ch and cv are horizontal and vertical components of the cycloid curve
        ch = (s   - math.sin(2*math.pi*s)/(2*math.pi))
        cv = (1.0 - math.cos(2*math.pi*s))/2.0
        state.foot[0:2, state.swg] = np.matrix((1-ch)*footstep[0:2, state.k-1] + ch*footstep[0:2, state.k+1]).transpose()
        state.foot[2, state.swg]   = cv*swing_height
        state.foot[3, state.swg]   = footstep[3,state.k-1] + ch*wrap_pi(footstep[3,state.k+1] - footstep[3, state.k-1])
    else:
        state.foot[0:4, state.swg] = np.matrix(footstep[0:4, state.k-1]).transpose()


    # update reference state using dcm equation
    state.p_com_ref = state.p_com_ref - (1/T)*(state.p_com_ref - state.p_dcm_ref)*dt
    state.p_dcm_ref = state.p_dcm_ref + (1/T)*(state.p_dcm_ref - state.p_zmp_ref)*dt

    # calc reference CoM velocity from reference DCM
    state.v_com_ref = (state.p_dcm_ref - state.p_com_ref)/T

    # reference state
    xref = np.matrix([state.p_com_ref[0, 0], state.v_com_ref[0, 0], state.p_com_ref[0, 1], state.v_com_ref[0, 1]])
    # reference input
    uref = np.matrix([state.p_zmp_ref[0], state.p_zmp_ref[1]])

    # state feedback
    state.u = (K*(xref - state.x).transpose()).transpose() + uref

    # state update with noise
    #v = np.matrix([0.0, 0.0, 0.0, 0.0])
    v = generate_noise(state.t)

    # update state
    xd = A*state.x.T + B*state.u.T + v.T
    state.x = state.x + (xd*dt).T


    state.t += dt
    state.t_total += dt

    if state.t >= step_duration:
        # reset local time
        state.t = 0
        # reset reference dcm and reference zmp

        #increment step index
        if state.k < N-1:
            state.k += 1
        # switch foot
        if state.sup == 0:
            state.sup = 1
            state.swg = 0
        else:
            state.sup = 0
            state.swg = 1

        state.p_dcm_ref = footstep[4:6, state.k]
        state.p_zmp_ref = footstep[6:8, state.k]


def ax_config():
    ax.set_title("Biped Walk", size=20)
    ax.set_xlabel("x", size = 14)
    ax.set_ylabel("y", size = 14)
    ax.set_zlabel("z", size = 14)
    ax.set_xlim([-0.2, 1.6])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([0.0, 1.5])
    #ax.view_init(-142.5, 30)

def draw_com(com):
    ax.plot(com[0], com[1], com[2], "o")

def draw_zmp(zmp):
    ax.plot(zmp[0], zmp[1], zmp[2], "o")

def draw_foot_r(foot):
    rxy = (foot[0,0] - f_size_len*math.cos(f_theta_rad + math.radians(foot[3,0])), foot[1,0] - f_size_len*math.sin(f_theta_rad + math.radians(foot[3,0])))
    patch_foot_r = patch.Rectangle(xy=rxy, angle=foot[3,0], width=foot_size[0], height=foot_size[1], ec='#000000', fill=False)
    ax.add_patch(patch_foot_r)
    art3d.pathpatch_2d_to_3d(patch_foot_r, z=foot[2,0], zdir="z")

def draw_foot_l(foot):
    lxy = (foot[0,1] - f_size_len*math.cos(f_theta_rad + math.radians(foot[3,1])), foot[1,1] - f_size_len*math.sin(f_theta_rad + math.radians(foot[3,1])))
    patch_foot_l = patch.Rectangle(xy=lxy, angle=foot[3,1], width=foot_size[0], height=foot_size[1], ec='#000000', fill=False)
    ax.add_patch(patch_foot_l)
    art3d.pathpatch_2d_to_3d(patch_foot_l, z=foot[2,1], zdir="z")

def draw_line(com, zmp):
    line = art3d.Line3D([com[0],zmp[0]], [com[1], zmp[1]], [com[2], zmp[2]], color='c')
    ax.add_line(line)





if __name__ == "__main__":
    main()


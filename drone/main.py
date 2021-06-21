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

from GLOBAL_VARIABLE import *
from calc_reference import *
from position_controller import *
from orientation_controller import *
from drone import *


sns.set()



# initial position
p0 = np.array([0., 0., 0.])
# initial velocity
v0 = np.array([0., 0., 0.])
# initial Euler angle
psi0 = np.array([0., 0., 0.])
# initial angular velocity
omega0 = np.array([0., 0., 0.])

dt = 0.01

fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')

class State:
    x0 = np.matrix([p0, v0, psi0, omega0])
    def __init__(self):
        self.x = State.x0
        self.t = 0

def main():
    state = State()
    anim = animation.FuncAnimation(fig, update, fargs=(state,), frames=range(256), interval=100)
    plt.show()


def update(frame, state):
    ax.cla()
    ax_config()
    simulation(state)
#    com = [state.x[0, 0], state.x[0, 2], h]
#    zmp = [state.u[0, 0], state.u[0, 1], 0.0]
#    draw_com(com)

def simulation(s):
    # reference trajectory
    (p_d, psi_d) = calc_reference(s.t)
    # position control
    (u_z, phi_d, theta_d) = position_controller(s.x, p_d)
    # orientation control
    u_rot = orientation_controller(s.x, np.array([phi_d, theta_d, psi_d]))

    u = np.array([u_z, u_rot])
    xd = drone(s.x, u)



def ax_config():
    ax.set_title("Drone", size=20)
    ax.set_xlabel("x", size = 14)
    ax.set_ylabel("y", size = 14)
    ax.set_zlabel("z", size = 14)
    ax.set_xlim([-0.2, 1.6])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([0.0, 1.5])

if __name__=="__main__":
    main()


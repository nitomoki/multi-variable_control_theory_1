import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as animation
import numpy as np
import math
from icecream import ic
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import seaborn as sns

from GLOBAL_VARIABLE import *
from calc_reference import *
from position_controller import *
from orientation_controller import *
from drone import *


sns.set()



# initial position
p0 = np.matrix([0., 0., 0.])
# initial velocity
v0 = np.matrix([0., 0., 0.])
# initial Euler angle
psi0 = np.matrix([0., 0., 0.])
# initial angular velocity
omega0 = np.matrix([0., 0., 0.])

dt = 0.01

fig = plt.figure(figsize=(16,16))
ax = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')

class State:
    #x0 = np.matrix(p0, v0, psi0, omega0)
    x0 = np.concatenate((p0, v0, psi0, omega0))
    def __init__(self):
        self.x = State.x0
        self.t = 0
        self.p_d = np.matrix([0.,0.,0.]).T
        self.p = np.matrix([0.,0.,0.]).T
        self.R = np.matrix([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])

def main():
    state = State()
    anim = animation.FuncAnimation(fig, update, fargs=(state,), frames=range(100), interval=100)
    anim.save("sample.gif", writer="pillow")
    plt.show()


def update(frame, s):
    ax.cla()
    ax2.cla()
    ax_config()
    simulation(s)
    draw_com(s.p)
    draw_com_ref(s.p_d)
    draw_roters(s.p, s.R)

def simulation(s):
    # reference trajectory
    (p_d, psi_d) = calc_reference(s.t)
    s.p_d = p_d
    # position control
    (u_direction, u_z, phi_d, theta_d) = position_controller(s.x, p_d)
    # orientation control
    u_rot = orientation_controller(s.x, np.matrix([phi_d, theta_d, psi_d]).T)
    u = np.concatenate((np.matrix([[u_z]]), u_rot))
    xd = drone(s.x, u)

    s.x += xd*dt
    s.t += dt

    s.p = s.x[0, :].T
    Psi = s.x[2, :].T
    s.R = calcR(Psi)

    draw_line(u_direction)
    draw_line(s.x[1].T, color='r')
    draw_line(s.R*np.matrix([0.,0.,1.]).T, color='g')


def ax_config():
    ax.set_title("Drone", size=20)
    ax.set_xlabel("x", size = 14)
    ax.set_ylabel("y", size = 14)
    ax.set_zlabel("z", size = 14)
    ax.set_xlim([-0.2, 1.6])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([0.0, 1.5])

    ax2.set_title("Drone pose", size=20)
    ax2.set_xlabel("x", size = 14)
    ax2.set_ylabel("y", size = 14)
    ax2.set_zlabel("z", size = 14)
    ax2.set_xlim([-1.0, 1.0])
    ax2.set_ylim([-1.0, 1.0])
    ax2.set_zlim([-1.0, 1.0])

def draw_com(pos):
    ax.plot(pos[0,0], pos[1,0], pos[2,0], "o")

def draw_com_ref(pos):
    ax.plot(pos[0,0], pos[1,0], pos[2,0], "o")

def draw_line(v, color='c'):
    line = art3d.Line3D([v[0,0],0.], [v[1,0], 0.], [v[2,0], 0.], color=color)
    ax2.add_line(line)

def draw_roters(p, R):
    pos1 = p + R*np.matrix([ L, 0,0]).T
    pos2 = p + R*np.matrix([-L, 0,0]).T
    pos3 = p + R*np.matrix([ 0, L,0]).T
    pos4 = p + R*np.matrix([ 0,-L,0]).T
    ax.plot(pos1[0,0], pos1[1,0], pos1[2,0], "o", color='k')
    ax.plot(pos2[0,0], pos2[1,0], pos2[2,0], "o", color='k')
    ax.plot(pos3[0,0], pos3[1,0], pos3[2,0], "o", color='k')
    ax.plot(pos4[0,0], pos4[1,0], pos4[2,0], "o", color='k')

if __name__=="__main__":
    main()


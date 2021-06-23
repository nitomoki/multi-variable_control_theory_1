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

animation = False

# initial position
p0 = np.matrix([0., 0., 0.])
# initial velocity
v0 = np.matrix([0., 0., 0.])
# initial Euler angle
psi0 = np.matrix([0., 0., 0.])
# initial angular velocity
omega0 = np.matrix([0., 0., 0.])

dt = 0.01

result_x = []
result_y = []
result_z = []
result_x_d =[]
result_y_d =[]
result_z_d =[]

fig = plt.figure(figsize=(16,8))
ax = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')

if animation == False:
    fig_result = plt.figure(figsize=(8,8))
    ax_resurt = fig_result.add_subplot(111, projection='3d')

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

    if animation:
        anim = animation.FuncAnimation(fig, update, fargs=(state,), frames=range(200), interval=100)
        #anim.save("sample.gif", writer="pillow")
        plt.show()

    if animation == False:
        for frame in range(1300):
            update(frame, state)
            ic(frame)
        ax_resurt.plot(result_x, result_y, result_z, ".", color='b')
        ax_resurt.plot(result_x_d, result_y_d, result_z_d, "-", color='#ff7f00')
        plt.show()


    ic()


def update(frame, s):
    #ic(frame)
    ax.cla()
    ax2.cla()
    ax_config()
    simulation(s)
    draw_com(s.p)
    draw_com_ref(s.p_d)
    draw_roters(s.p, s.R)

    if animation == False:
        result_x.append(s.p[0,0])
        result_y.append(s.p[1,0])
        result_z.append(s.p[2,0])
        result_x_d.append(s.p_d[0,0])
        result_y_d.append(s.p_d[1,0])
        result_z_d.append(s.p_d[2,0])
        ic(s.p)
        ic(s.p_d)


def simulation(s):
    # reference trajectory
    (p_d, psi_d) = calc_reference(s.t, shape='eight')
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

    draw_line(u_direction, label='desired')
    draw_line(s.x[1].T, color='r', label='velocity')
    draw_line(s.R*np.matrix([0.,0.,1.]).T, color='g', label='drone upper')
    ax2.legend(bbox_to_anchor=(1,1), loc='upper right')


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

    if animation == False:
        ax_resurt.set_title("Result", size=20)
        ax_resurt.set_xlabel("x", size = 14)
        ax_resurt.set_ylabel("y", size = 14)
        ax_resurt.set_zlabel("z", size = 14)
        ax_resurt.set_xlim([-0.2, 1.6])
        ax_resurt.set_ylim([-1.0, 1.0])
        ax_resurt.set_zlim([0.0, 1.5])

def draw_com(pos):
    ax.plot(pos[0,0], pos[1,0], pos[2,0], "o")

def draw_com_ref(pos):
    ax.plot(pos[0,0], pos[1,0], pos[2,0], "o", color='#ff7f00')

def draw_line(v, color='c', label=''):
    line = art3d.Line3D([v[0,0],0.], [v[1,0], 0.], [v[2,0], 0.], color=color, label=label)
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


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
sns.set()

playing = False
# parameters
h = 1.0
g = 9.8
step_duration = 0.5
foot_size = np.matrix([0.2, 0.2])
swing_height = 0.03
# constatnt of LIPM
T = math.sqrt(h/g)
# time step
dt = 0.01
# animation_speed
animation_speed = 1.0
# generate walking pattern

# number of footsteps

# variables
sup = 0 # current support foot 0:right 1:left
swg = 1 # current swing foot   0:right 1:left
k = 2 # current footstep index
t = 0 # elapsed time since last foot exchange
t_total = 0 # total elapsed time
foot = np.matrix([[0, 0],\
                  [0, 0],\
                  [0, 0],\
                  [0, 0]])
                 # foot pose
                 # rows: x y z theta
                 # columns: right left
p_com = np.matrix([0, 0])
v_com = np.matrix([0, 0])
a_com = np.matrix([0, 0])
p_zmp = np.matrix([0, 0])
p_com_ref = np.matrix([0, 0])
# p_dcm_ref = # function footstep() needed
# p_zmp_ref = 
x = np.matrix([p_com[0, 0], v_com[0, 0], p_com[0, 1], v_com[0, 1]])
u = p_zmp

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
R = np.diag([10000, 10000])
K, P, e = lqr(A, B, Q, R)
ic(K)
ic(P)
ic(e)

fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')

pi = np.pi

n = 256

t = np.linspace(0, 0.2*pi, n)

x = np.cos(30*t)*0.3
y = np.sin(30*t)*0.3
z = t
def ax_config():
    ax.set_title("Surface Plot", size=20)
    ax.set_xlabel("x", size = 14)
    ax.set_ylabel("y", size = 14)
    ax.set_zlabel("z", size = 14)
    ax.set_xlim([-0.2, 1.6])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([0, 0.6])

def update(frame):
    ax.cla()
    ax_config()
    ax.plot(frame*0.01, 0, 0, "o")

if playing:
    anim = animation.FuncAnimation(fig, update, frames=range(256), interval=100)
    plt.show()
#ax.plot(x, y, z, color="red")


import matplotlib.pyplot as plt
import matplotlib.patches as patch
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d

from scipy.stats import multivariate_normal



fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Surface Plot", size=20)
ax.set_xlabel("x", size = 14)
ax.set_ylabel("y", size = 14)
ax.set_zlabel("z", size = 14)
ax.set_xticks([-1.0, -0.5, 0.0, 0.5, 1.0])
ax.set_yticks([-1.0, -0.5, 0.0, 0.5, 1.0])

line = art3d.Line3D([0,3],[0,3],[0,3], color='blue')
ax.add_line(line)

pi = np.pi
n = 256

t = np.linspace(-6*pi, 6*pi, n)

x = np.cos(t)
y = np.sin(t)
z = t

ax.plot(x, y, z, color="red")

plt.show()

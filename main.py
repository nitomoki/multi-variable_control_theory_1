import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0,1,100)
y = x**2

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(x, y)

plt.show()

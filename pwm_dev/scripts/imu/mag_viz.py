""" Simple Mag Field Plot """
import numpy as np
from matplotlib import pyplot as plt

M = np.load('/tmp/mag.npy')
plt.plot(M[:,0], M[:,1])
plt.axis('equal')
plt.show()


import numpy as np
from matplotlib import pyplot as plt

M = np.load('/tmp/mag_true.npy')
plt.plot(M[:,0], M[:,1])
plt.axis('equal')
plt.show()


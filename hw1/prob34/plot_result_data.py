import matplotlib.pyplot as plt
import numpy as np

results = np.loadtxt('prob3_data.txt', delimiter=',', skiprows=1)
plt.plot(results[:,0], results[:,1])
plt.xlabel('x')
plt.ylabel('y')
plt.title('Numerically simulated trajetory')
plt.savefig('prob3.eps')
plt.show()
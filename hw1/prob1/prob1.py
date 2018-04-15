import numpy as np
import matplotlib.pyplot as plt

t1 = 10
t2 = 5
v0 = 0.5


def run_simulation(R):
    C1 = v0 / (4 * R) * (1 / t1 - 1 / t2)
    C2 = v0 / 2 * (1 / t1 + 1 / t2)
    t = np.linspace(0, 10, 100)
    # phi = C1 * t ** 2
    x = C2 / (2 * C1) * np.sin(C1 * t ** 2)
    y = C2 / (2 * C1) * (1 - np.cos(C1 * t ** 2))
    return x, y


R = 0.12
x, y = run_simulation(R)
plt.plot(x, y)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory of differentially steered robot')
plt.show()
plt.savefig('prob1.eps')

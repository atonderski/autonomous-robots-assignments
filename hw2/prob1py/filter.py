import time

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import spline

np.random.seed(12312312)

WHEEL_RADIUS = 0.23
RADIUS = 0.5
GPS_NOISE_STD_DEV = 0.5
ODOMETER_NOISE_STD_DEV = 0.2
FREQ = 20
t_max = 1000
N = t_max * FREQ
Ns = 100
USE_HACK = False
USE_NOISE_HACK = True
NO_SLIP = False

GPS_NOISE_VAR = GPS_NOISE_STD_DEV ** 2
ODOMETER_NOISE_VAR = ODOMETER_NOISE_STD_DEV ** 2
DT = 1 / FREQ


class Filter:
    def __init__(self):
        self.m = 4
        self.n = 5
        self.xHat = np.zeros((self.n, 1))

        # Jacobian of h
        self.H = np.zeros((self.m, self.n))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 3] = self.H[3, 4] = 1
        # error
        self.P = np.identity(self.n)
        # process noise
        pos_noise = 0.00001
        angle_noise = 0.00001
        speed_noise = 0.00001
        self.Q = np.diag([pos_noise, pos_noise, angle_noise, speed_noise, speed_noise])
        # measurment noise
        self.R = np.diag([GPS_NOISE_VAR, GPS_NOISE_VAR, ODOMETER_NOISE_VAR, ODOMETER_NOISE_VAR])

    def update(self, z, no_gps, no_odo):
        F_old = self._F(self.xHat)
        self.xHat = self._f(self.xHat)

        self.P = F_old.dot(self.P).dot(F_old.T) + self.Q

        R = self.R.copy()
        if USE_NOISE_HACK:
            if no_gps:
                R[0, 0] = R[1, 1] = 99999999
            if no_odo:
                R[2, 2] = R[3, 3] = 99999999
        G = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + R))
        self.xHat += G.dot(z - self._h(self.xHat))

        I = np.identity(self.n)
        self.P = (I - G.dot(self.H)).dot(self.P)

        return self.xHat[0], self.xHat[1], self.xHat[3], self.xHat[4]

    def _h(self, x):
        return np.array(
            [x[0],
             x[1],
             x[3],
             x[4]])

    def _f(self, x):
        v = 0.5 * (x[3] + x[4])
        return np.array([
            x[0] + DT * v * np.cos(x[2]),
            x[1] + DT * v * np.sin(x[2]),
            x[2] - DT * 0.5 * (x[3] - x[4]) / RADIUS,
            x[3],
            x[4]
        ])

    def _F(self, x):
        v = 0.5 * (x[3] + x[4])
        cosPhi = np.cos(x[2])
        sinPhi = np.sin(x[2])

        return np.array([
            [1, 0, -v * sinPhi * DT, 0.5 * DT * cosPhi, 0.5 * DT * cosPhi],
            [0, 1, v * cosPhi * DT, 0.5 * DT * sinPhi, 0.5 * DT * sinPhi],
            [0, 0, 1, -0.5 * DT / RADIUS, 0.5 * DT / RADIUS],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])


t = np.linspace(0, t_max, N)

# Kinda random wheel paths (no slip)
omega_l = 0.8 + 1.2 * np.sin(0.001 * t) * np.sin(0.012 * t)
omega_r = 0.8 + 1.2 * np.sin(0.0015 * t) * np.cos(0.02 * t)

# Very straight path
# omega_l = 1 + 0.05 * np.sin(0.001 * t)
# omega_r = 1 + 0.05 * np.sin(0.0011 * t)

# Too hard
# omega_l = 0.8 + 1.1 * np.sin(0.001 * t) * np.sin(0.012 * t) + 0.1 * np.sin(0.1*t)
# omega_r = 0.7 + 1.15 * np.sin(0.0015 * t) * np.cos(0.02 * t) + 0.15 * np.sin(0.15*t)

# Add random slip to the wheel speeds
slip_sampling = np.linspace(0, t_max, Ns)
slip_l = 0.9 + 0.1 * np.random.rand(Ns)
slip_r = 0.9 + 0.1 * np.random.rand(Ns)
s_l = spline(slip_sampling, slip_l, t)
s_r = spline(slip_sampling, slip_r, t)
real_omega_l = omega_l * s_l
real_omega_r = omega_r * s_r

# Plot wheel speeds
plt.plot(t, omega_l)
plt.plot(t, omega_r)
plt.plot(t, real_omega_l)
plt.plot(t, real_omega_r)
plt.xlabel(r'$t$ (s)')
plt.ylabel(r'$\omega$ (rad/s)')
plt.legend((r'$\omega_L$', r'$\omega_R$', r'$\omega_L$ + slip', r'$\omega_R$ + slip'))
plt.savefig('velocities_1.eps')
plt.show()


def step(omega_l, omega_r, x, y, phi):
    phi += -WHEEL_RADIUS * (omega_l - omega_r) / (2 * RADIUS) * DT
    v = WHEEL_RADIUS * (omega_l + omega_r) / 2
    x += v * np.cos(phi) * DT
    y += v * np.sin(phi) * DT
    return x, y, phi


xs, ys = 0 * t[:], 0 * t[:]
noslip_xs, noslip_ys = 0 * t[:], 0 * t[:]
measured_xs, measured_ys = 0 * t[:], 0 * t[:]
gps_xs, gps_ys = 0 * t[:], 0 * t[:]
errors, gps_errors = 0 * t[:], 0 * t[:]

x, y, phi = 0, 0, 0
noslip_x, noslip_y, noslip_phi = 0, 0, 0
prev_vl, prev_vr = 0, 0
gps_x, gps_y = 0, 0

kalman_filter = Filter()
z = np.zeros((4, 1))
for i, _ in enumerate(t):
    no_gps = True
    no_odo = True
    if i % 20 == 0:
        # gps_noise = np.random.multivariate_normal(np.array([0,0]), np.diag([GPS_NOISE_VAR, GPS_NOISE_VAR]))
        gps_noise = [np.random.normal(0, GPS_NOISE_STD_DEV), np.random.normal(0, GPS_NOISE_STD_DEV)]
        if NO_SLIP:
            gps_x = noslip_x + gps_noise[0]
            gps_y = noslip_y + gps_noise[1]
        else:
            gps_x = x + gps_noise[0]
            gps_y = y + gps_noise[1]
        z[0] = gps_x
        z[1] = gps_y
        no_gps = False
    elif USE_HACK:
        z[0] = measured_xs[i - 1]
        z[1] = measured_ys[i - 1]
    if i % 2 == 0:
        z[2] = WHEEL_RADIUS * (omega_l[i] + np.random.normal(0, ODOMETER_NOISE_STD_DEV))
        z[3] = WHEEL_RADIUS * (omega_r[i] + np.random.normal(0, ODOMETER_NOISE_STD_DEV))
        no_odo = False
    elif USE_HACK:
        z[2] = prev_vl
        z[3] = prev_vr

    measured_xs[i], measured_ys[i], prev_vl, prev_vr = kalman_filter.update(z, no_gps, no_odo)

    x, y, phi = step(real_omega_l[i], real_omega_r[i], x, y, phi)
    xs[i] = x
    ys[i] = y

    noslip_x, noslip_y, noslip_phi = step(omega_l[i], omega_r[i], noslip_x, noslip_y, noslip_phi)
    noslip_xs[i] = noslip_x
    noslip_ys[i] = noslip_y

    gps_xs[i], gps_ys[i] = gps_x, gps_y

    if NO_SLIP:
        errors[i] = np.linalg.norm(np.array([noslip_x, noslip_y]) - np.array([measured_xs[i], measured_ys[i]]))
        gps_diff = np.array([noslip_x, noslip_y]).flatten() - np.array([gps_x, gps_y]).flatten()
        gps_errors[i] = np.linalg.norm(gps_diff)
    else:
        errors[i] = np.linalg.norm(np.array([x, y]) - np.array([measured_xs[i], measured_ys[i]]))
        gps_diff = np.array([x, y]).flatten() - np.array([gps_x, gps_y]).flatten()
        gps_errors[i] = np.linalg.norm(gps_diff)

plt.figure()
plt.plot(xs, ys, 'b', lw=2)
plt.plot(noslip_xs, noslip_ys, 'g--')
plt.plot(measured_xs, measured_ys, 'r', alpha=0.7)
plt.plot(gps_xs, gps_ys, 'yo', zorder=-1, ms=1)
plt.xlabel(r'$x$ (m)')
plt.ylabel(r'$y$ (m)')
plt.legend(('true path', 'path without slip', 'estimated position', 'gps readings'))
plt.title(r'Paths with $\sigma_{gps}=%.1f$. $\sigma_{odometer}=%.1f$' % (GPS_NOISE_STD_DEV, ODOMETER_NOISE_STD_DEV))
plt.savefig('paths_%.1f_%.1f.eps' % (GPS_NOISE_STD_DEV, ODOMETER_NOISE_STD_DEV))
# time.sleep(5)
plt.show()

print("Total gps error: %.2f" % np.sum(gps_errors))
print("Average gps error: %.2f" % np.mean(gps_errors))
print("Max gps error: %.2f" % np.max(gps_errors))
print("Total error: %.2f" % np.sum(errors))
print("Average error: %.2f" % np.mean(errors))
print("Max error: %.2f" % np.max(errors))
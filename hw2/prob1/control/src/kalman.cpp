/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(double dt, double gpsNoiseStdDev, double wheelNoiseStdDev, double radius)
        : m{4}, n{5}, t{0}, DT{dt}, Q{Eigen::MatrixXd::Identity(n, n)},
          R{Eigen::MatrixXd::Identity(m, m)}, P{Eigen::MatrixXd::Identity(n, n)}, xHat{Eigen::VectorXd::Zero(n)},
          z{Eigen::VectorXd::Zero(m)}, noGPS{true}, noWheelSpeed{true}, RADIUS{radius} {
    // Set correct noise values
    Q *= .05;
    R(0, 0) = gpsNoiseStdDev * gpsNoiseStdDev;
    R(1, 1) = gpsNoiseStdDev * gpsNoiseStdDev;
    R(2, 2) = wheelNoiseStdDev * wheelNoiseStdDev;
    R(3, 3) = wheelNoiseStdDev * wheelNoiseStdDev;
}

void KalmanFilter::setInitState(double x, double y, double phi, double vl, double vr){
    xHat << x, y, phi, vl, vr;
}

void KalmanFilter::setGPS(double x, double y) {
    z(0) = x;
    z(1) = y;
    noGPS = false;
}

void KalmanFilter::setWheelSpeed(double vL, double vR) {
    z(2) = vL;
    z(3) = vR;
    noWheelSpeed = false;
}

void KalmanFilter::update() {
    Eigen::MatrixXd F_old = F();
    xHat = f();
    Eigen::MatrixXd H_new = H();

    // If we don't have any new measurements we increase the uncertainty to maximum for
    Eigen::MatrixXd R_new{R};
    if (noGPS) {
        R_new(0, 0) = std::numeric_limits<double>::max();
        R_new(1, 1) = std::numeric_limits<double>::max();
    }
    if (noWheelSpeed) {
        R_new(2, 2) = std::numeric_limits<double>::max();
        R_new(3, 3) = std::numeric_limits<double>::max();
    }

    P = F_old * P * F_old.transpose() + Q;
    Eigen::MatrixXd G = P * H_new.transpose() * (H_new * P * H_new.transpose() + R_new).inverse();
    xHat += G * (z - h());
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    P = (I - G * H_new) * P;

    t += DT;
    z *= 0.0;
    noWheelSpeed = true;
    noGPS = true;
}

Eigen::VectorXd KalmanFilter::f() {
    double v = 0.5 * (xHat(3) + xHat(4));
    Eigen::VectorXd f(n);
    f << xHat(0) + DT * v * std::cos(xHat(2)),
            xHat(1) + DT * v * std::sin(xHat(2)),
            xHat(2) - DT * 0.5 * (xHat(3) - xHat(4)) / RADIUS,
            xHat(3),
            xHat(4);
    return f;
}

Eigen::MatrixXd KalmanFilter::F() {
    double v = 0.5 * (xHat(3) + xHat(4));
    double cosPhi = std::cos(xHat(2));
    double sinPhi = std::sin(xHat(2));

    Eigen::MatrixXd F(n, n);
    F << 1, 0, -v * sinPhi * DT, 0.5 * DT * cosPhi, 0.5 * DT * cosPhi,
            0, 1, v * cosPhi * DT, 0.5 * DT * sinPhi, 0.5 * DT * sinPhi,
            0, 0, 1, -0.5 * DT / RADIUS, 0.5 * DT / RADIUS,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    return F;
}

Eigen::VectorXd KalmanFilter::h() {
    Eigen::VectorXd h(m);
    h << xHat(0),
            xHat(1),
            xHat(3),
            xHat(4);
    return h;
}

Eigen::MatrixXd KalmanFilter::H() {
    Eigen::MatrixXd H(m, n);
    H << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    return H;
}

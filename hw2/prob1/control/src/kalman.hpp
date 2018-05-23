// turn off the specific warning. Can also use "-Wall"
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Weffc++"

#include "Eigen/Dense"
// turn the warnings back on
#pragma GCC diagnostic pop

#pragma once

class KalmanFilter {

public:
    KalmanFilter(double dt, double gpsNoiseStdDev, double wheelNoiseStdDev, double radius);

    void setInitState(double x, double y, double phi, double vl, double vr);

    void update();

    Eigen::VectorXd &state() { return xHat; };

    double time() { return t; };

    void setGPS(double x, double y);

    void setWheelSpeed(double vL, double vR);

private:
    Eigen::VectorXd f();

    Eigen::MatrixXd F();

    Eigen::VectorXd h();

    Eigen::MatrixXd H();

    // System dimensions
    int m, n;

    double t;

    double const DT;

    Eigen::MatrixXd Q, R, P;
    Eigen::VectorXd xHat, z;

    bool noGPS, noWheelSpeed;

    double const RADIUS;
};

#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <iostream>
#include <Eigen/Dense>

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

double polyeval(Eigen::VectorXd coeffs, double x);

double get_curvature(Eigen::VectorXd coeffs, double x);

double get_slope(Eigen::VectorXd coeffs, double x);

#endif

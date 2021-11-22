//
// Created by enes on 20.07.2021.
//

#include "eigen_utils.h"

double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) { result += coeffs[i] * pow(x, i); }
    return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


double get_curvature(Eigen::VectorXd coeffs, double x){
    return pow((1 + pow((2*coeffs[2]*x + coeffs[1]), 2)), 1.5) / (2 * coeffs[2]);
}

double get_slope(Eigen::VectorXd coeffs, double x){
    return (-coeffs[1]/2*coeffs[2])*x;
}

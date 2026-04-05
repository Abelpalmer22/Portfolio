#pragma once
#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using Mat12x4 = Eigen::Matrix<double, 12, 4>;
using Mat4 = Eigen::Matrix<double, 4, 4>;

struct Params {
    double m;
    double g;
    double l;
    Mat12 Q;
    Mat4 R;
    Mat12 V;
    Mat12 W;
    double c_tau;
    double Jx;
    double Jy;
    double Jz;
    double dx;
    double dynamics;
    double dz;
    double cruising_speed;
    Eigen::Matrix<double, 3, 3> F;
    Eigen::VectorXd x;
    Mat12 C;
    double tracking_dist;
    int N;
}


#include "dynamics.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

Vec12 quadrotor_ode(const Vec12& x, const Vec4& u, const Params& p) {
    double px = x(0);
    double py = x(1);
    double pz = x(2);
    double vx = x(3);
    double vy = x(4);
    double vz = x(5);
    double phi = x(6);
    double theta = x(7);
    double psi = x(8);
    double pp = x(9);
    double qq = x(10);
    double rr = x(11);

    double f1 = vx;
    double f2 = vy;
    double f3 = vz;

    double T = u(0) + u(1) + u(2) + u(3);
    double sphi = std::sin(phi);
    double cphi = std::cos(phi);
    double stheta = std::sin(theta);
    double ctheta = std::cos(theta);
    double spsi = std::sin(psi);
    double cpsi = std::cos(psi);
    double ttheta = std::tan(theta);

    double f4 = (T / p.m) * (cpsi * stheta * cphi + spsi * sphi) - (p.dx / p.m) * vx;
    double f5 = (T / p.m) * (spsi * stheta * cphi - cpsi * sphi) - (p.dy / p.m) * vy;
    double f6 = (T / p.m) * (ctheta * cphi) - p.g - (p.dz / p.m) * vz;

    double f7 = pp + qq * sphi * ttheta + rr * cphi * ttheta;
    double f8 = qq * cphi - rr * sphi;
    double f9 = qq * (sphi / ctheta) + rr * (cphi / ctheta);

    double tau_x = p.l * (u(1) - u(3));
    double tau_y = p.l * (u(2) - u(0));
    double tau_z = p.c_tau * (u(0) - u(1) + u(2) - u(3));

    double f10 = (tau_x - (p.Jz - p.Jy) * qq * rr) / p.Jx;
    double f11 = (tau_y - (p.Jx - p.Jz) * pp * rr) / p.Jy;
    double f12 = (tau_z - (p.Jy - p.Jx) * pp * qq) / p.Jz;
    Vec12 res;
    res << f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12;
    return res;
}

Vec12 rk4step(const Vec12& x, const Vec4& u, const Params& p, const double dt) {
    Vec12 k1 = quadrotor_ode(x, u, p);
    Vec12 k2 = quadrotor_ode(x + 0.5 * dt * k1, u, p);
    Vec12 k3 = quadrotor_ode(x + 0.5 * dt * k2, u, p);
    Vec12 k4 = quadrotor_ode(x + dt * k3, u, p);
    return x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
}

std::pair<Mat12, Mat12x4> linearize(const Vec12& x0, const Vec4& u0, const Params& p, double eps) {
    Mat12 A;
    Mat12x4 B;

    for (int i = 0; i < 12; ++i) {
        Vec12 dx = Vec12::Zero();
        dx(i) = eps;

        const Vec12 f_plus  = quadrotor_ode(x0 + dx, u0, p);
        const Vec12 f_minus = quadrotor_ode(x0 - dx, u0, p);

        A.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }

    for (int i = 0; i < 4; ++i) {
        Vec4 du = Vec4::Zero();
        du(i) = eps;

        const Vec12 f_plus  = quadrotor_ode(x0, u0 + du, p);
        const Vec12 f_minus = quadrotor_ode(x0, u0 - du, p);

        B.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }

    return {A, B};
}
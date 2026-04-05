#pragma once
#include "types.hpp"

Vec12 quadrotor_ode(const Vec12& x, const Vec4& u, const Params& p);
Vec12 rk4step(const Vec12& x, const Vec4& u, const Params& p, const double dt);
std::pair<Mat12, Mat12x4> linearize(const Vec12& x0, const Vec4& u0, const Params& p, double eps = 1e-6);
#include "constants.hpp"

Params default_params() {
    Params p;
    p.m = 0.5;
    p.g = 9.81;
    p.l = 0.175;
    p.c_tau = 0.0036;
    p.Jx = 2.32e-3;
    p.Jy = 2.32e-3;
    p.Jz = 4.00e-3;
    p.dx = 0.01;
    p.dy = 0.01;
    p.dz = 0.01;
    p.cruising_speed = 50.0;

    p.V = 1e-7 * Mat12::Identity();
    p.W = 1e-3 * Mat12::Identity();
    p.F = 1e-3 * Mat3::Identity();

    Vec12 qdiag;
    qdiag << 40, 40, 40, 10, 10, 10, 7, 7, 0.01, 1, 1, 0.5;
    p.Q = qdiag.asDiagonal();

    p.R = 0.1 * Mat4::Identity();
    p.C = Mat12::Identity();

    p.tracking_dist = 1.0;
    p.N = 2000;

    return p;
}
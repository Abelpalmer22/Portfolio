#include "control.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <random>
#include "dynamics.hpp"

std::pair<Mat12, Mat12x4> discretize(const Mat12& A, const Mat12x4& B, double dt) {
    Eigen::Matrix<double, 16, 16> M = Eigen::Matrix<double, 16, 16>::Zero();

    M.block<12,12>(0,0) = A;
    M.block<12,4>(0,12) = B;

    Eigen::Matrix<double, 16, 16> Md = (M * dt).exp();
    Mat12 Ad = Md.block<12,12>(0,0);
    Mat12x4 Bd = Md.block<12,4>(0,12);
    
    return {Ad, Bd};
}

Eigen::MatrixXd riccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, int maxiters=1000, double tol=1e-9) {
    Eigen::MatrixXd P = Q;
    for (int i = 0; i < maxiters; ++i) {
        Eigen::MatrixXd P_next = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        if ((P_next - P).norm() < tol) {return P_next;}
        P = P_next;
    }
    return P;
}

Eigen::MatrixXd dlqr(const Eigen::MatrixXd& Ad, const Eigen::MatrixXd& Bd, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    Eigen::MatrixXd P = riccati(Ad, Bd, Q, R);
    Eigen::MatrixXd K = (Bd.transpose() * P * Bd + R).ldlt().solve(Bd.transpose() * P * Ad);
    return K;
}

Eigen::MatrixXd dkalman(const Eigen::MatrixXd& Ad, const Eigen::MatrixXd& C, const Eigen::MatrixXd& V, const Eigen::MatrixXd& W) {
    Eigen::MatrixXd P = riccati(Ad.transpose(), C.transpose(), V, W);
    Eigen::MatrixXd L = (C * P * C.transpose() + W).ldlt().solve(C * P).transpose();
    return L;
}

// struct ScheduleData {
//     std::string name;
//     Eigen::Matrix<double, 12, 1> x_eq;
//     Eigen::Matrix<double, 4, 1> u_eq;

//     Eigen::Matrix<double, 12, 12> A;
//     Eigen::Matrix<double, 12, 4> B;
//     Eigen::Matrix<double, 12, 12> Ad;
//     Eigen::Matrix<double, 12, 4> Bd;
//     Eigen::Matrix<double, 4, 12> K;
//     Eigen::Matrix<double, 12, 12> L;
// }

std::pair<std::map<std::string, Vec12>, std::map<std::string, Vec4>> schedule_points(Params& p) {
    std::map<std::string, Vec12> points;
    std::map<std::string, Vec4> controls;
    std::vector<Eigen::Vector3d> dirs;

    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                dirs.emplace_back(i,j,k);
            }
        }
    }

    for (Eigen::Vector3d dir : dirs) {
        if (dir.isZero()) {
            std::string key = "hover";
            Vec12 x_trim = Vec12::Zero();
            Vec4 u_trim = (p.m * p.g / 4) * Vec4::Ones();
            points[key] = x_trim;
            controls[key] = u_trim;
            continue;
        }
        Eigen::Vector3d d_unit = dir / dir.norm();
        Eigen::Vector3d velocity = p.cruising_speed * d_unit;
        double vx = velocity(0);
        double vy = velocity(1);
        double vz = velocity(2);
        double Fx = p.dx * vx;
        double Fy = p.dy * vy;
        double Fz = p.m * p.g + p.dz * vz;
        double T = std::sqrt(Fx*Fx + Fy*Fy + Fz*Fz);
        double phi = std::asin(-Fy / T);
        double theta = std::atan2(Fx, Fz);
        Vec12 x_trim;
        x_trim << -1.0, 0.0, 0.0,
                  vx, vy, vz,
                  phi, theta, 0.0,
                  0.0, 0.0, 0.0;
        Vec4 u_trim = (T / 4) * Eigen::Vector4d::Ones();
        std::string key = std::format("dir_{}_{}_{}", dir(0), dir(1), dir(2));
        points[key] = x_trim;
        controls[key] = u_trim;
    }
    return {points, controls};
}

std::map<std::string, std::map<std::string, Eigen::MatrixXd>> build_schedule_bank(std::map<std::string, Vec12>& points, std::map<std::string, Vec4>& controls, double dt, Params p) {
    std::map<std::string, std::map<std::string, Eigen::MatrixXd>> bank;

    for (const auto& [name, xeq] : points) {
        Vec12 xeq = points[name];
        Vec4 ueq = controls[name];
        auto [A, B] = linearize(xeq, ueq, p);
        auto [Ad, Bd] = discretize(A, B, dt);
        auto K = dlqr(Ad, Bd, p.Q, p.R);
        auto L = dkalman(Ad, p.C, p.V, p.W);

        bank[name] = {
            {"x_eq", xeq},
            {"u_eq", ueq},
            {"A", A},
            {"B", B},
            {"Ad", Ad},
            {"Bd", Bd},
            {"K", K},
            {"L", L}
        }
    }
    return bank;
}

std::string choose_schedule(const Vec12& x_hat, const std::map<std::string, std::map<std::string, Eigen::MatrixXd>>& bank) {
    std::string best_name;
    double best_dist = std::numeric_limits<double>::infinity();
    
    for (const auto& [name, data] : bank) {
        Eigen::VectorXd xe = data.at("x_eq");
        double dist = (x_hat.segment(3, 9) - xe.segment(3, 9)).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_name = name;
        }
    }
    return best_name;
}

Eigen::VectorXd sample_mvn(const Eigen::MatrixXd& V) {
    static std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> dist(0.0, 1.0);
    int n = V.rows();
    Eigen::VectorXd z(n);
    for (int i = 0; i < n; i++) {
        z(i) = dist(gen);
    }
    Eigen::MatrixXd L = V.llt().matrixL();
    return L * z;
}

auto simulate_gain_scheduled_lqg(const Vec12& x0, const Eigen::MatrixXd& target_positions, const std::map<std::string, std::map<std::string, Eigen::MatrixXd>>& schedule_bank, double dt, Params p) {
    int N = target_positions.cols();
    Eigen::MatrixXd X_true = Eigen::MatrixXd::Zero(12, N);
    Eigen::MatrixXd X_hat  = Eigen::MatrixXd::Zero(12, N);
    Eigen::MatrixXd U      = Eigen::MatrixXd::Zero(4, N);
    Eigen::MatrixXd Y_meas = Eigen::MatrixXd::Zero(12, N);
    Eigen::MatrixXd Y_targ = Eigen::MatrixXd::Zero(3, N);
    auto x_true = x0;
    auto x_hat = x0;

    std::string sigma = choose_schedule(x_hat, schedule_bank);
    Vec4 u_prev = schedule_bank.at(sigma).at("u_eq");

    for (int k = 0; k < N; k++) {
        x_true = rk4step(x_true, u_prev, p, dt);
        x_true += sample_mvn(p.V);

        Vec12 y = p.C * x_true + sample_mvn(p.W);

        Vec3 target_pos = target_positions.col(k);
        Vec3 tracker_pos_true = x_true.head(3);
        Vec3 y_target = target_pos - tracker_pos_true + sample_mvn(p.F).head<3>();

        sigma = choose_schedule(x_hat, schedule_bank);
        auto data = schedule_bank[sigma];

        Eigen::MatrixXd Ad = data.at("Ad");
        Eigen::MatrixXd Bd = data.at("Bd");
        Eigen::MatrixXd K = data.at("K");
        Eigen::MatrixXd L = data.at("L");
        Vec12 x_eq = data.at("x_eq");
        Vec4 u_eq = data.at("u_eq");

        Vec12 x_pred = x_eq + Ad * (x_hat - x_eq) + Bd * (u_prev - u_eq);
        x_hat = x_pred + L * (y - p.C * x_pred);

        Vec12 x_d = Vec12::Zero();
        Vec3 lag;
        lag << p.tracking_dist << 0.0 << 0.0;
        x_d.head(3) = x_hat.head(3) + y_target - lag;
        
        Vec4 u = u_eq - K * (x_hat - x_d);

        X_true.col(k) = x_true;
        X_hat.col(k) = x_hat;
        U.col(k) = u;
        Y_meas.col(k) = y;
        Y_targ.col(k) = y_target;
        u_prev = u;
    }
    return {X_true, X_hat, U, Y_meas, Y_targ};
}










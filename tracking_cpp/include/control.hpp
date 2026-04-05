#pragma once
#include "types.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <tuple>

std::pair<Mat12, Mat12x4> discretize(const Mat12& A, const Mat12x4& B, double dt);
Eigen::MatrixXd riccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, int maxiters=1000, double tol=1e-9);
Eigen::MatrixXd dlqr(const Eigen::MatrixXd& Ad, const Eigen::MatrixXd& Bd, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
Eigen::MatrixXd dkalman(const Eigen::MatrixXd& Ad, const Eigen::MatrixXd& C, const Eigen::MatrixXd& V, const Eigen::MatrixXd& W);

std::pair<std::map<std::string, Vec12>, std::map<std::string, Vec4>>
schedule_points(const Params& p);

std::map<std::string, std::map<std::string, Eigen::MatrixXd>>
build_schedule_bank(const std::map<std::string, Vec12>& points,
                    const std::map<std::string, Vec4>& controls,
                    double dt, const Params& p);

std::string choose_schedule(const Vec12& x_hat,
    const std::map<std::string, std::map<std::string, Eigen::MatrixXd>>& bank);

Eigen::VectorXd sample_mvn(const Eigen::MatrixXd& V);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
simulate_gain_scheduled_lqg(const Vec12& x0,
                            const Eigen::MatrixXd& target_positions,
                            const std::map<std::string, std::map<std::string, Eigen::MatrixXd>>& schedule_bank,
                            double dt, const Params& p);
#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

namespace {

void print(const std::string& title, const Eigen::Ref<const Eigen::MatrixXd>& M) {
    std::cout << title << "\n" << M << "\n\n";
}

}  // namespace

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // Example: 1D constant-velocity model, state x = [position, velocity]^T, observe position.
    const int n = 2;  // state dimension
    const int m = 1;  // measurement dimension

    const double dt = 0.1;

    Eigen::Matrix2d F;
    F << 1.0, dt, 0.0, 1.0;

    Eigen::Matrix2d Q;
    Q << 1e-4, 0.0, 0.0, 1e-4;

    Eigen::Matrix<double, 1, 2> H;
    H << 1.0, 0.0;

    Eigen::Matrix<double, 1, 1> R;
    R << 0.25;

    Eigen::Vector2d x_post;
    x_post << 0.0, 0.0;

    Eigen::Matrix2d P_post;
    P_post << 1.0, 0.0, 0.0, 1.0;

    Eigen::Vector<double, 1> z;
    z << 1.2;

    std::cout << "=== Discrete-time Kalman filter: one time step ===\n\n";

    std::cout << "--- Model (held fixed for this step) ---\n";
    print("F (state transition)", F);
    print("Q (process noise covariance)", Q);
    print("H (measurement matrix)", H);
    print("R (measurement noise covariance)", R);
    print("z (measurement vector)", z);

    std::cout << "--- After previous step (posterior at k-1) ---\n";
    print("x_{k-1|k-1}", x_post);
    print("P_{k-1|k-1}", P_post);

    // ----- Predict -----
    Eigen::Vector2d x_prior = F * x_post;
    Eigen::Matrix2d P_prior = F * P_post * F.transpose() + Q;

    std::cout << "--- Predict (time update) ---\n";
    print("x_{k|k-1} = F * x_{k-1|k-1}", x_prior);
    print("P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q", P_prior);

    // ----- Update -----
    Eigen::Vector<double, 1> y = z - H * x_prior;
    Eigen::Matrix<double, 1, 1> S = H * P_prior * H.transpose() + R;
    Eigen::Matrix<double, 2, 1> K = P_prior * H.transpose() * S.inverse();

    Eigen::Vector2d x_new = x_prior + K * y;
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d P_new = (I - K * H) * P_prior;

    std::cout << "--- Update (measurement update) ---\n";
    print("Innovation y = z - H * x_{k|k-1}", y);
    print("Innovation covariance S = H * P_{k|k-1} * H^T + R", S);
    print("Kalman gain K = P_{k|k-1} * H^T * S^{-1}", K);
    print("Posterior mean x_{k|k} = x_{k|k-1} + K * y", x_new);
    print("Posterior covariance P_{k|k} = (I - K*H) * P_{k|k-1}", P_new);

    std::cout << "=== End of step ===\n";
    return 0;
}

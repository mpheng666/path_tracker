// #include "../src/state_space_model.cpp"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace path_tracker
{
    struct LinearSystem
    {
        Eigen::Matrix<double, 3, 3> A_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 2, 1> B_;

        Eigen::Matrix<double, 2, 3> getMatrixB(double theta, double dt)
        {
            Eigen::Matrix<double, 2, 3> B;
            B << cos(theta) * dt, -sin(theta) * dt, 0.0,  //
                sin(theta) * dt, cos(theta) * dt, 0.0,    //
                0.0, 0.0, dt;                             //
            return B;
        }
    };

    class LinearQuadraticRegulator
    {
        public:
        LinearQuadraticRegulator() { init(); }

        Eigen::Matrix<double, 3, 1>
        stepControl(const Eigen::Matrix<double, 3, 1>& current_state,
                    const Eigen::Matrix<double, 3, 1>& target_state)
        {
            auto error = target_state - current_state;
            const size_t N {50};
            std::vector<Eigen::Matrix<double, 3, 3>> P(N + 1);
            P.at(N) = Q_;
            for (int i = N; i >= 0; --i)
            {
                // P[i-1] = Q + ATP[i]A – (ATP[i]B)(R + BTP[i]B)^-1(BTP[i]A)
                P.at(i - 1) =
                    Q_ + (linear_system_.A_.transpose() * P.at(i) * linear_system_.A_) -
                    (linear_system_.A_ * P.at(i) * linear_system_.B_) *
                        (R_ +
                         (linear_system_.B_.transpose() * P.at(i) * linear_system_.B_))
                            .inverse() *
                        (linear_system_.B_.transpose() * P.at(i) * linear_system_.A_);
            }
            std::vector<Eigen::Matrix<double, 3, 3>> K(N);
            for (int i = 0; i < N; ++i)
            {
                // K[i] = -(R + BTP[i+1]B)^-1BTP[i+1]A
                K.at(i) =
                    -(R_ +
                      linear_system_.B_.transpose() * P.at(i + 1) * linear_system_.B_)
                         .inverse() *
                    (linear_system_.B_.transpose() * P.at(i + 1) * linear_system_.A_);
            }
            std::vector<Eigen::Matrix<double, 3, 3>> u(N);
            for (int i = 0; i < N; ++i)
            {
                u.at(i) = K.at(i) * error;
            }
            Eigen::Matrix<double, 3, 1> optimal_control = u.at(N - 1);
            return optimal_control;
        }

        private:
        LinearSystem linear_system_;
        Eigen::Matrix<double, 3, 3> Q_;
        Eigen::Matrix<double, 3, 3> R_;
        double time_interval_ {};

        void init()
        {
            Q_ = Eigen::Matrix3d::Identity();
            R_ << 0.1, 0.0,  //
                0.0, 0.1;    //
        }
    };
}  // namespace path_tracker

int main() { return 0; }
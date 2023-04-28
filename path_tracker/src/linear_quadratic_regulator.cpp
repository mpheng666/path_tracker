// #include "../src/state_space_model.cpp"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace path_tracker
{
    class LinearSystem
    {
        public:
        LinearSystem() { init(); };

        void init()
        {
            state_current_ << 0.0, 0.0, 0.0;
            state_previous_ = state_current_;
            A_              = Eigen::Matrix3d::Identity();
        }

        Eigen::Matrix<double, 3, 3> getMatrixB(double theta, double dt)
        {
            Eigen::Matrix<double, 3, 3> B;
            B << cos(theta) * dt, -sin(theta) * dt, 0.0,  //
                sin(theta) * dt, cos(theta) * dt, 0.0,    //
                0.0, 0.0, dt;                             //
            return B;
        }

        void compute(const Eigen::Matrix<double, 3, 1>& input)
        {
            B_             = getMatrixB(state_current_(2, 0), 0.1);
            state_current_ = A_ * state_previous_ + B_ * input;
            std::cout << "current state: \n" << state_current_ << "\n";
            state_previous_ = state_current_;
        }

        Eigen::Matrix<double, 3, 3> A_;
        Eigen::Matrix<double, 3, 3> B_;
        Eigen::Matrix<double, 3, 1> state_current_;
        Eigen::Matrix<double, 3, 1> state_previous_;
    };

    class LinearQuadraticRegulator
    {
        public:
        LinearQuadraticRegulator(Eigen::Matrix<double, 3, 3>& A,
                                 Eigen::Matrix<double, 3, 3>& B/* ,
                                 Eigen::Matrix<double, 3, 3>& Q,
                                 Eigen::Matrix<double, 3, 3>& R */):
            A_(A),
            B_(B)/* , Q_(Q), R_(R) */
        {
            init();
        }

        Eigen::Matrix<double, 3, 1>
        stepControl(const Eigen::Matrix<double, 3, 1>& current_state,
                    const Eigen::Matrix<double, 3, 1>& target_state)
        {
            auto error = current_state - target_state;
            const int N {49};
            std::vector<Eigen::Matrix<double, 3, 3>> P(N + 1);
            P.at(N) = Q_;
            for (int i = N; i > 0; --i)
            {
                // discrete-time algebraic ricatti equation
                // P[i-1] = Q + ATP[i]A – (ATP[i]B)(R + BTP[i]B)^-1(BTP[i]A)
                P.at(i - 1) =
                    Q_ + (A_.transpose() * P.at(i) * A_) -
                    (A_.transpose() * P.at(i) * B_) *
                        (R_ +
                         (B_.transpose() * P.at(i) * B_))
                            .inverse() *
                        (B_.transpose() * P.at(i) * A_);
            }
            std::vector<Eigen::Matrix<double, 3, 3>> K(N);
            for (int i = 0; i < static_cast<int>(N); ++i)
            {
                // K[i] = -(R + BTP[i+1]B)^-1BTP[i+1]A
                K.at(i) =
                    -(R_ +
                      B_.transpose() * P.at(i + 1) * B_)
                         .inverse() *
                    (B_.transpose() * P.at(i + 1) * A_);
            }
            std::vector<Eigen::Matrix<double, 3, 1>> u(N);
            for (int i = 0; i < static_cast<int>(N); ++i)
            {
                u.at(i) = K.at(i) * error;
            }
            Eigen::Matrix<double, 3, 1> optimal_control;
            optimal_control = u.at(N - 1);
            return optimal_control;
        }

        private:
        Eigen::Matrix<double, 3, 3> A_;
        Eigen::Matrix<double, 3, 3> B_;
        Eigen::Matrix<double, 3, 3> Q_;
        Eigen::Matrix<double, 3, 3> R_;
        double time_interval_ {1.0};

        void init()
        {
            Q_ = Eigen::Matrix3d::Identity();
            R_ << 0.5, 0.0, 0.0,  //
                0.0, 0.5, 0.0,    //
                0.0, 0.0, 0.5;    //
        }
    };
}  // namespace path_tracker

int main()
{
    path_tracker::LinearSystem omni_robot;
    Eigen::Matrix<double, 3, 1> current_state;
    current_state << 0, 0, 0;
    Eigen::Matrix<double, 3, 1> target_state;
    target_state << 2, -3, -3.4;
    while ((current_state - target_state).norm() > 0.1)
    {
        path_tracker::LinearQuadraticRegulator lqr(omni_robot.A_, omni_robot.B_);
        auto optimal_control = lqr.stepControl(current_state, target_state);
        std::cout << "optimal control: \n" << optimal_control << "\n";
        omni_robot.compute(optimal_control);
        current_state = omni_robot.state_current_;
    }

    return 0;
}
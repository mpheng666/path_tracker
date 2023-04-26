#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace path_tracker
{
    class StateSpaceModel
    {
        public:
        StateSpaceModel() { init(); }

        void init()
        {
            // linear time-invariant system in state-space form
            state_current_ << 0.0, 0.0, 0.0;
            state_previous_ = state_current_;
            A_matrix_       = Eigen::Matrix3d::Identity();
            input_matrix_ << 2.0, 0, 0;
        }

        void compute(const Eigen::Matrix<double, 3, 1>& input)
        {
            state_current_ =
                A_matrix_ * state_previous_ + getMatrixB(state_current_(2,0), 1.0) * input;
            std::cout << "current state: \n" << state_current_ << "\n";
            state_previous_ = state_current_;
        }

        private:
        static constexpr unsigned int STATE_NUM_ {3};
        static constexpr unsigned int CONTROL_NUM_ {3};
        Eigen::Matrix<double, STATE_NUM_, 1> state_current_;
        Eigen::Matrix<double, STATE_NUM_, 1> state_previous_;
        Eigen::Matrix<double, STATE_NUM_, STATE_NUM_> A_matrix_;
        Eigen::Matrix<double, STATE_NUM_, CONTROL_NUM_> B_matrix_;
        Eigen::Matrix<double, CONTROL_NUM_, 1> input_matrix_;

        Eigen::Matrix<double, STATE_NUM_, CONTROL_NUM_> getMatrixB(double theta,
                                                                   double dt)
        {
            B_matrix_ << cos(theta) * dt, -sin(theta) * dt, 0.0, sin(theta) * dt,
                cos(theta) * dt, 0.0, 0.0, 0.0, dt;
            return B_matrix_;
        }
    };
}  // namespace path_tracker

int main()
{
    path_tracker::StateSpaceModel model;
    char input = 0;
    std::array<double, 3> input_matrix {0.0, 0.0, 0.0};
    while (input != 'q')
    {
        std::cout << "Enter one of the key w,a,s,d,z,c to move or q to quit: \n";
        std::cin >> input;
        switch (input)
        {
            case 'w':
                input_matrix.at(0) += 0.2;
                break;
            case 's':
                input_matrix.at(0) -= 0.2;
                break;
            case 'a':
                input_matrix.at(1) += 0.2;
                break;
            case 'd':
                input_matrix.at(1) -= 0.2;
                break;
            case 'z':
                input_matrix.at(2) += 0.2;
                break;
            case 'c':
                input_matrix.at(2) -= 0.2;
                break;
            default:
                std::cout << "Please input valid input \n";
                break;
        }
        std::cout << "Current input: ";
        for(const auto& input : input_matrix)
        {
            std::cout << input << ",";
        }
        std::cout << "\n";
        model.compute({input_matrix.at(0), input_matrix.at(1), input_matrix.at(2)});
    }
    return 0;
}
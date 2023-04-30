#include "../src/linear_quadratic_regulator.cpp"

int main(int argc, char ** argv)
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

  return 0;
}

#include <eigen3/Eigen/Dense>

template <int state_dim, int input_dim, int observe_dim>
class KF
{
public:
    Eigen::Matrix<float, state_dim, 1> x_pre;
    Eigen::Matrix<float, state_dim, 1> x_last;
    Eigen::Matrix<float, state_dim, state_dim> state_trans_matrix;
    Eigen::Matrix<float, input_dim, 1> u;
    Eigen::Matrix<float, state_dim, input_dim> input_matrix;
    Eigen::Matrix<float, state_dim, state_dim> cov_matrix_last;
    Eigen::Matrix<float, state_dim, state_dim> cov_matrix_pre;
    Eigen::Matrix<float, state_dim, state_dim> process_noise;
    Eigen::Matrix<float, observe_dim, observe_dim> observe_noise;
    Eigen::Matrix<float, state_dim, observe_dim> kalman_gain;
    Eigen::Matrix<float, observe_dim, state_dim> observe_matrix;

    Eigen::Matrix<float, state_dim, 1> update(Eigen::Matrix<float, observe_dim, 1> z, Eigen::Matrix<float, input_dim, 1> u);
};

template <int state_dim, int input_dim, int observe_dim>
Eigen::Matrix<float, state_dim, 1> KF<state_dim, input_dim, observe_dim>::update(Eigen::Matrix<float, observe_dim, 1> z, Eigen::Matrix<float, input_dim, 1> u)
{
    x_pre = state_trans_matrix * x_last + input_matrix * u;
    cov_matrix_pre = state_trans_matrix * cov_matrix_last * (state_trans_matrix).transpose() + process_noise;
    kalman_gain = cov_matrix_pre * (observe_matrix).transpose() * (observe_matrix * cov_matrix_pre * (observe_matrix).transpose() + observe_noise).inverse();
    x_last = x_pre + kalman_gain * (z - observe_matrix * x_pre);
    cov_matrix_last = (Eigen::Matrix<float, state_dim, state_dim>::Identity() - kalman_gain * observe_matrix) * cov_matrix_pre;
    return x_last;
}

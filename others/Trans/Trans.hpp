#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>

class Trans
{

public:
    Trans(std::string from, std::string to, Eigen::Matrix3f transform_matrix, Eigen::Vector3f _t_in_from);
    Trans(std::string from, std::string to, Eigen::Vector3f ypr_from_at_to, Eigen::Vector3f _t_in_from);
    Trans(std::string from, std::string to, cv::Point3f ypr_from_at_to, Eigen::Vector3f _t_in_from);
    Trans(std::string from, std::string to);

    Eigen::Vector3f transform(Eigen::Vector3f v);
    Eigen::Vector3f transform_inv(Eigen::Vector3f v);
    void update_trans(Eigen::Matrix3f transform_matrix);
    void update_trans(Eigen::Vector3f ypr_from_at_to);
    void update_trans(cv::Point3f ypr_from_at_to);

    void update_shift(Eigen::Vector3f _t);
    void update_shift(cv::Point3f _t);

    std::string from;
    std::string to;

    Eigen::Matrix3f rotation_matrix;
    Eigen::Matrix3f rotation_matrix_inv;
    Eigen::Vector3f t;

    //private:
    static Eigen::Matrix3f EulerAngle2RotationMatrix(const Eigen::Vector3f &eulerAngle);
};

#include "Trans.hpp"
#include <iostream>

using namespace Eigen;

Eigen::Matrix3f Trans::EulerAngle2RotationMatrix(const Eigen::Vector3f &eulerAngle) //YPR x-Yaw->y-Pit->r-Roll in base
{
    Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle(2), Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle(1), Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle(0), Vector3f::UnitZ()));
    // std::cout << "roll M\n"
    //           << rollAngle.matrix() << std::endl;
    // std::cout << "pitch M\n"
    //           << pitchAngle.matrix() << std::endl;
    // std::cout << "yaw M\n"
    //           << yawAngle.matrix() << std::endl;
    return (yawAngle * pitchAngle * rollAngle).matrix();
}

Trans::Trans(std::string _from, std::string _to)
{
    from = _from;
    to = _to;
    t=Vector3f::Zero();
    rotation_matrix=Matrix3f::Identity();
    rotation_matrix_inv=Matrix3f::Identity();
}

Trans::Trans(std::string _from, std::string _to, Eigen::Matrix3f _transform_matrix, Eigen::Vector3f _t_in_from)
{
    from = _from;
    to = _to;
    rotation_matrix = _transform_matrix;
    rotation_matrix_inv = rotation_matrix.transpose();
    t = _t_in_from;
}

Trans::Trans(std::string _from, std::string _to, Eigen::Vector3f _ypr_from_at_to, Eigen::Vector3f _t_in_from)
{
    from = _from;
    to = _to;
    rotation_matrix = EulerAngle2RotationMatrix(_ypr_from_at_to);
    rotation_matrix_inv = rotation_matrix.transpose();
    t = _t_in_from;
}

Trans::Trans(std::string _from, std::string _to, cv::Point3f _ypr_from_at_to, Eigen::Vector3f _t_in_from)
{
    from = _from;
    to = _to;
    Vector3f v(_ypr_from_at_to.x, _ypr_from_at_to.y, _ypr_from_at_to.z);
    rotation_matrix = EulerAngle2RotationMatrix(v);
    rotation_matrix_inv = rotation_matrix.transpose();
    t = _t_in_from;
}

Eigen::Vector3f Trans::transform(Eigen::Vector3f v)
{
    return rotation_matrix * (v + t);
}
Eigen::Vector3f Trans::transform_inv(Eigen::Vector3f v)
{
    return rotation_matrix_inv * v - t;
}

void Trans::update_trans(Eigen::Matrix3f transform_matrix)
{
    rotation_matrix = transform_matrix;
    rotation_matrix_inv = rotation_matrix.transpose();
}

void Trans::update_trans(Eigen::Vector3f ypr_from_at_to)
{
    rotation_matrix = EulerAngle2RotationMatrix(ypr_from_at_to);
    rotation_matrix_inv = rotation_matrix.transpose();
}

void Trans::update_trans(cv::Point3f ypr_from_at_to)
{
    Vector3f v(ypr_from_at_to.x, ypr_from_at_to.y, ypr_from_at_to.z);
    rotation_matrix = EulerAngle2RotationMatrix(v);
    rotation_matrix_inv = rotation_matrix.transpose();
}

void Trans::update_shift(Eigen::Vector3f _t)
{
    t = _t;
}
void Trans::update_shift(cv::Point3f _t)
{
    t = Vector3f(_t.x, _t.y, _t.z);
}

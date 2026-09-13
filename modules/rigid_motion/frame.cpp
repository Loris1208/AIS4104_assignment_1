#include "rigid_motion/frame.h"
#include "rigid_motion/screw.h"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace ais4104::rigid_motion {

//TASK: 1d
//REFERENCE: Formula of the rotation matrix page 72, MR pre-print 2019
Eigen::Matrix3d rotate_x(double radians)
{
    Eigen::Matrix3d r =  Eigen::Matrix3d::Zero();
    r << 1.0, 0.0, 0.0, 0.0, std::cos(radians),-std::sin(radians),0.0,std::sin(radians),std::cos(radians);// << for 3x3 matrix
    return r;
}

//TASK: 1e
//REFERENCE: Formula of the rotation matrix page 72, MR pre-print 2019
Eigen::Matrix3d rotate_y(double radians)
{
    Eigen::Matrix3d r =  Eigen::Matrix3d::Zero();
    r << std::cos(radians), 0.0, std::sin(radians), 0.0, 1.0, 0.0,-std::sin(radians), 0.0, std::cos(radians);
    return r;
}

//TASK: 1f
//REFERENCE: Formula of the rotation matrix page 72, MR pre-print 2019
Eigen::Matrix3d rotate_z(double radians)
{
    Eigen::Matrix3d r =  Eigen::Matrix3d::Zero();
    r << std::cos(radians), -std::sin(radians), 0.0, std::sin(radians), std::cos(radians), 0.0, 0.0, 0.0, 1.0;
    return r;
}

//TASK: 1g
//REFERENCE: Equation (3.16) page 65, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    Eigen::Matrix3d r = Eigen::Matrix3d::Zero();
    r.col(0) = x;
    r.col(1) = y;
    r.col(2) = z;
    return r;
}

//TASK: 1h
//REFERENCE: Equation (3.51) page 82, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    Eigen::Matrix3d m;
    m = Eigen::Matrix3d::Identity() +  std::sin(radians) * skew_symmetric(axis) + (1-std::cos(radians)) * skew_symmetric(axis) * skew_symmetric(axis);
    return m;
}

//TASK: 1i Implement the special cases for Euler ZYX and XYZ using rotate_x, rotate_y, and rotate_z
//REFERENCE: First formula in section B.1 page 577, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_euler(const Eigen::Vector3d &e, praxis::axis_order order)
{
    if(order == praxis::axis_order::zyx)
    {
        return rotate_z(e[0]) * rotate_y(e[1]) * rotate_x(e[2]);
    }
    else if(order == praxis::axis_order::xyz)
    {
        return rotate_x(e[0]) * rotate_y(e[1]) * rotate_z(e[2]);
    }

    const std::array<std::uint8_t, 3> axes = praxis::axis_indices(order);
    return rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[0]), e[0]) *
           rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[1]), e[1]) *
           rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[2]), e[2]);
}

//TASK: 1j -- Implement the special case for calculating the Euler ZYX.
//REFERENCE: Equation (B.3, B.4 and B.5) page 579, MR pre-print 2019
Eigen::Vector3d euler_from_rotation_matrix(const Eigen::Matrix3d &r, praxis::axis_order order)
{
    if(order == praxis::axis_order::zyx)
    {
        Eigen::Vector3d e = Eigen::Vector3d::Zero();

        e[0] = atan2(r(1,0), r(0,0));
        e[1] = atan2(-r(2,0),sqrt(r(0,0) * r(0,0) + r(1,0) * r(1,0)));
        e[2] = atan2(r(2,1), r(2,2));

        return e; //not sure
    }

    const std::array<std::uint8_t, 3> axes = praxis::axis_indices(order);

    return r.eulerAngles(axes[0], axes[1], axes[2]);
}

//TASK: 2b
//REFERENCE: Equation (3.63) page 88, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_transform(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d r = tf.block<3,3>(0,0);
    return r;
}

//TASK: 2c
//REFERENCE: Equation (3.63) page 88, MR pre-print 2019
Eigen::Matrix4d transformation_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen:: Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3,3>(0,0) = r;
    t.block<3,1>(0,3) = p;
    return t;
}

//TASK: 2d
//REFERENCE: Equation (3.63) page 88, MR pre-print 2019
Eigen::Matrix4d transformation_matrix_from_position(const Eigen::Vector3d &p)
{
    Eigen:: Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3,1>(0,3) = p;
    return t;
}

//TASK: 2e
//REFERENCE: Equation (3.63) page 88, MR pre-print 2019
Eigen::Matrix4d transformation_matrix_from_rotation(const Eigen::Matrix3d &r)
{
    Eigen:: Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3,3>(0,0) = r;
    return t;
}

//TASK: 2f
//REFERENCE: Equation (3.64) page 88, MR pre-print 2019
Eigen::Matrix4d inverse(const Eigen::Matrix4d &tf)
{
    Eigen:: Matrix3d r = rotation_matrix_from_transform(tf);
    Eigen:: Vector3d p = tf.block<3,1>(0,3);
    Eigen:: Matrix4d inv_tf = Eigen::Matrix4d::Identity();
    inv_tf.block<3,3>(0,0) = r.transpose();
    inv_tf.block<3,1>(0,3) = -r.transpose() * p;
    return inv_tf;
}

}
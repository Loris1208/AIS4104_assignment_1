#include "rigid_motion/frame.h"
#include "rigid_motion/screw.h"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace ais4104::rigid_motion {

//TASK: 1d
//REFERENCE:
Eigen::Matrix3d rotate_x(double radians)
{
    Eigen::Matrix3d r =  Eigen::Matrix3d::Zero();
    r << 1.0, 0.0, 0.0,0.0,
    0.0, std::cos(radians),-std::sin(radians),
    0.0,std::sin(radians),std::sin(radians);// << for 3x3 matrix

    return Eigen::Matrix3d::Zero();


}

//TASK: 1e
//REFERENCE: 
Eigen::Matrix3d rotate_y(double radians)
{
    return Eigen::Matrix3d::Zero();
}

//TASK: 1f
//REFERENCE:
Eigen::Matrix3d rotate_z(double radians)
{
    return Eigen::Matrix3d::Zero();
}

//TASK: 1g
//REFERENCE: 
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    //x.col(0) = 0.0;
    //y.col(0) = 0.0;
    //z.col(0) = 0.0;
    return Eigen::Matrix3d::Zero();

}

//TASK: 1h
//REFERENCE: 
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    return Eigen::Matrix3d::Zero();
}

//TASK: 1i Implement the special cases for Euler ZYX and XYZ using rotate_x, rotate_y, and rotate_z
//REFERENCE: 
Eigen::Matrix3d rotation_matrix_from_euler(const Eigen::Vector3d &e, praxis::axis_order order)
{
    if(order == praxis::axis_order::zyx)
    {
        return Eigen::Matrix3d::Zero();
    }
    else if(order == praxis::axis_order::xyz)
    {
        return Eigen::Matrix3d::Zero();
    }

    const std::array<std::uint8_t, 3> axes = praxis::axis_indices(order);
    return rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[0]), e[0]) * 
           rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[1]), e[1]) *
           rotation_matrix_from_axis_angle(Eigen::Vector3d::Unit(axes[2]), e[2]);
}

//TASK: 1j -- Implement the special case for calculating the Euler ZYX.
//REFERENCE: 
Eigen::Vector3d euler_from_rotation_matrix(const Eigen::Matrix3d &r, praxis::axis_order order)
{
    if(order == praxis::axis_order::zyx)
    {
        return Eigen::Vector3d::Zero();
    }

    const std::array<std::uint8_t, 3> axes = praxis::axis_indices(order);

    return r.eulerAngles(axes[0], axes[1], axes[2]);
}

//TASK: 2b
//REFERENCE: 
Eigen::Matrix3d rotation_matrix_from_transform(const Eigen::Matrix4d &tf)
{
    return Eigen::Matrix3d::Zero();
}

//TASK: 2c
//REFERENCE: 
Eigen::Matrix4d transformation_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen:: Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3,3>(0,0) = r;
    t.block<3,1>(0,0) = p;

    return t;
    //return Eigen::Matrix4d::Zero();
}

//TASK: 2d
//REFERENCE: 
Eigen::Matrix4d transformation_matrix_from_position(const Eigen::Vector3d &p)
{
    Eigen:: Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3,1>(0,3) = p;
    return t;
    //return Eigen::Matrix4d::Zero();
}

//TASK: 2e
//REFERENCE: 
Eigen::Matrix4d transformation_matrix_from_rotation(const Eigen::Matrix3d &r)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 2f
//REFERENCE: 
Eigen::Matrix4d inverse(const Eigen::Matrix4d &tf)
{
    return Eigen::Matrix4d::Zero();
}

}

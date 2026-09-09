#include "rigid_motion/frame.h"
#include "rigid_motion/screw.h"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <numbers>

namespace ais4104::rigid_motion {

//TASK: 1b
//REFERENCE:
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    return Eigen::Matrix3d::Zero();
}

//TASK: 1c
//REFERENCE:
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
    return Eigen::Vector3d::Zero();
}

//TASK: 3b
//REFERENCE:
praxis::expected<Eigen::Vector6d, praxis::refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3c
//REFERENCE:
Eigen::Vector6d screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{   Eigen::Vector6d screw;
    screw.head<3>() = w;
    screw.head<3>() = v;

    return screw;
    //return Eigen::Vector6d::Zero();
}

//TASK: 3d
//REFERENCE:
Eigen::Vector6d twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return Eigen::Vector6d::Zero();
}

//TASK: 3e
//REFERENCE:
praxis::expected<Eigen::Vector6d, praxis::refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3f
//REFERENCE:
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3g
//REFERENCE:
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_transform(const Eigen::Matrix4d &tf)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3h
//REFERENCE:
praxis::expected<Eigen::Vector6d, praxis::refusal> adjoint_map(const Eigen::Vector6d &t, const Eigen::Matrix4d &tf)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3i
//REFERENCE:
Eigen::Matrix4d twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3j
//REFERENCE:
Eigen::Matrix4d twist_matrix_from_twist(const Eigen::Vector6d &t)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3k
//REFERENCE:
Eigen::Matrix3d matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians)
{
    return Eigen::Matrix3d::Zero();
}

//TASK: 3l
//REFERENCE:
Eigen::Matrix4d matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3m
//REFERENCE:
Eigen::Matrix4d matrix_exponential_screw(const Eigen::Vector6d &s, double theta_radians)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3n
//REFERENCE:
praxis::expected<std::pair<Eigen::Vector3d, double>, praxis::refusal> matrix_logarithm_so3(const Eigen::Matrix3d &r)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3o
//REFERENCE:
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3_rp(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 3p
//REFERENCE:
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3(const Eigen::Matrix4d &tf)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}

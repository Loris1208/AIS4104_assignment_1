#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_SCREW_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_SCREW_H

#include "rigid_motion/alias.h"

#include <praxis/rigid_motion/types.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <Eigen/Core>

#include <utility>

namespace ais4104::rigid_motion {

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v);
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m);

praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_transform(const Eigen::Matrix4d &tf);
praxis::expected<Eigen::Vector6d, praxis::refusal> adjoint_map(const Eigen::Vector6d &t, const Eigen::Matrix4d &tf);

Eigen::Vector6d twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
praxis::expected<Eigen::Vector6d, praxis::refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity);
Eigen::Matrix4d twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
Eigen::Matrix4d twist_matrix_from_twist(const Eigen::Vector6d &t);

Eigen::Vector6d screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
praxis::expected<Eigen::Vector6d, praxis::refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);

Eigen::Matrix3d matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians);
Eigen::Matrix4d matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians);
Eigen::Matrix4d matrix_exponential_screw(const Eigen::Vector6d &s, double theta_radians);

praxis::expected<std::pair<Eigen::Vector3d, double>, praxis::refusal> matrix_logarithm_so3(const Eigen::Matrix3d &r);
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3_rp(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3(const Eigen::Matrix4d &tf);

}

#endif

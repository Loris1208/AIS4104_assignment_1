#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_FRAME_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_FRAME_H

#include <praxis/rigid_motion/types.h>
#include <praxis/rigid_motion/axis_order.h>

#include <Eigen/Core>

namespace ais4104::rigid_motion {

Eigen::Vector3d euler_from_rotation_matrix(const Eigen::Matrix3d &r, praxis::axis_order order);

Eigen::Matrix3d rotate_x(double radians);
Eigen::Matrix3d rotate_y(double radians);
Eigen::Matrix3d rotate_z(double radians);
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z);
Eigen::Matrix3d rotation_matrix_from_euler(const Eigen::Vector3d &e, praxis::axis_order order);
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians);
Eigen::Matrix3d rotation_matrix_from_transform(const Eigen::Matrix4d &tf);

Eigen::Matrix4d transformation_matrix_from_position(const Eigen::Vector3d &p);
Eigen::Matrix4d transformation_matrix_from_rotation(const Eigen::Matrix3d &r);
Eigen::Matrix4d transformation_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);
Eigen::Matrix4d inverse(const Eigen::Matrix4d &tf);

}

#endif

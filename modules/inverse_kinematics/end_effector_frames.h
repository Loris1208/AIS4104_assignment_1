#ifndef HPP_GUARD_AIS4104_INVERSE_KINEMATICS_END_EFFECTOR_FRAMES_H
#define HPP_GUARD_AIS4104_INVERSE_KINEMATICS_END_EFFECTOR_FRAMES_H

#include "rigid_motion/alias.h"

#include <praxis/manipulator/kinematics.h>

#include <praxis/rigid_motion/frame.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

namespace ais4104::manipulator {

praxis::expected<Eigen::Matrix4d, praxis::refusal> tool_pose_from_flange_pose(const Eigen::Matrix4d &flange_pose, const Eigen::Matrix4d &tool_offset);

praxis::expected<Eigen::Matrix4d, praxis::refusal> flange_pose_from_tool_pose(const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &tool_pose,
                                                                              const Eigen::Matrix4d &tool_offset);

praxis::expected<Eigen::Vector3d, praxis::refusal> position_from_pose(const Eigen::Matrix4d &pose);

praxis::expected<Eigen::Matrix3d, praxis::refusal> orientation_from_pose(const Eigen::Matrix4d &pose);

praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_flange_pose(const praxis::manipulator::kinematics &solver, const Eigen::Matrix4d &flange_pose, const Eigen::VectorXd &j0);

praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_pose(const praxis::manipulator::kinematics &solver, const praxis::rigid_motion::frame_ops &frames,
                                                                 const Eigen::Matrix4d &tool_pose, const Eigen::VectorXd &j0, const Eigen::Matrix4d &tool_offset);

}

#endif

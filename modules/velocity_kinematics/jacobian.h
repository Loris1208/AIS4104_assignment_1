#ifndef HPP_GUARD_AIS4104_VELOCITY_KINEMATICS_JACOBIAN_H
#define HPP_GUARD_AIS4104_VELOCITY_KINEMATICS_JACOBIAN_H

#include "rigid_motion/alias.h"

#include <praxis/manipulator/types.h>
#include <praxis/manipulator/kinematics.h>

#include <praxis/rigid_motion/frame.h>
#include <praxis/rigid_motion/screw.h>
#include <praxis/rigid_motion/types.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <span>

namespace ais4104::manipulator {

praxis::expected<praxis::manipulator::jacobian, praxis::refusal> space_jacobian(const praxis::rigid_motion::screw_ops &screw, std::span<const Eigen::Vector6d> space_screws,
                                                                                const Eigen::VectorXd &theta);

praxis::expected<praxis::manipulator::jacobian, praxis::refusal> body_jacobian(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                               const praxis::manipulator::forward_kinematics_ops &forward, const Eigen::Matrix4d &m,
                                                                               std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta);

}

#endif

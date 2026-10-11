#ifndef HPP_GUARD_AIS4104_FORWARD_KINEMATICS_KINEMATICS_H
#define HPP_GUARD_AIS4104_FORWARD_KINEMATICS_KINEMATICS_H

#include "rigid_motion/alias.h"

#include <praxis/manipulator/types.h>

#include <praxis/rigid_motion/frame.h>
#include <praxis/rigid_motion/screw.h>
#include <praxis/rigid_motion/types.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <span>
#include <vector>

namespace ais4104::manipulator {

praxis::expected<Eigen::Matrix4d, praxis::refusal> forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const Eigen::Matrix4d &m,
                                                                      std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta);

praxis::expected<std::vector<Eigen::Vector6d>, praxis::refusal> body_screws_from_space(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                                       const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws);

praxis::expected<Eigen::Matrix4d, praxis::refusal> body_forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                           const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta);


}

#endif

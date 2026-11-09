#ifndef HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_PATH_H
#define HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_PATH_H

#include "rigid_motion/alias.h"

#include <praxis/trajectory/path.h>

#include <praxis/rigid_motion/frame.h>
#include <praxis/rigid_motion/screw.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

namespace ais4104::trajectory {

praxis::expected<Eigen::VectorXd, praxis::refusal> joint_straight_line(const Eigen::VectorXd &start, const Eigen::VectorXd &end, double s);

praxis::expected<Eigen::Matrix4d, praxis::refusal> screw(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                         const Eigen::Matrix4d &end, double s);

praxis::expected<Eigen::Matrix4d, praxis::refusal> decoupled(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                             const Eigen::Matrix4d &end, double s);

}

#endif

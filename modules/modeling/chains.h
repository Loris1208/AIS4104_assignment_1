#ifndef HPP_GUARD_AIS4104_MODELING_CHAINS_H
#define HPP_GUARD_AIS4104_MODELING_CHAINS_H

#include "rigid_motion/alias.h"

#include <praxis/rigid_motion/frame.h>
#include <praxis/rigid_motion/screw.h>

#include <Eigen/Core>

#include <vector>
#include <utility>

namespace ais4104::modeling {

std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_kr6r900sixx_chain(const praxis::rigid_motion::screw_ops &screw);
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_lbr_med14_r820_chain(const praxis::rigid_motion::screw_ops &screw);

Eigen::Matrix4d hardcoded_kr6r900sixx_curved_rod_tool_offset(const praxis::rigid_motion::frame_ops &frames);
Eigen::Matrix4d hardcoded_kr6r900sixx_humerus_tool_offset(const praxis::rigid_motion::frame_ops &frames);

}

#endif

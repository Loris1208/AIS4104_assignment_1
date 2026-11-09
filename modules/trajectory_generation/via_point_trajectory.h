#ifndef HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_VIA_POINT_TRAJECTORY_H
#define HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_VIA_POINT_TRAJECTORY_H

#include <praxis/trajectory/trajectory.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <span>
#include <memory>

namespace ais4104::trajectory {

praxis::expected<std::unique_ptr<praxis::trajectory::trajectory_generator>, praxis::refusal> joint_space_via_points(std::span<const praxis::trajectory::via_point> via_points);

}

#endif

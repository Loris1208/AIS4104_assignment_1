#ifndef HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_VIA_POINTS_H
#define HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_VIA_POINTS_H

#include "rigid_motion/alias.h"

#include <praxis/trajectory/trajectory.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

namespace ais4104::trajectory {

// The coefficients a_j0 to a_j3 of Lynch & Park, Modern Robotics, eq. (9.25), one entry per joint, in the time since the segment's first via point.
struct cubic_segment
{
    Eigen::VectorXd a0;
    Eigen::VectorXd a1;
    Eigen::VectorXd a2;
    Eigen::VectorXd a3;
};

praxis::expected<cubic_segment, praxis::refusal> segment_coefficients(const praxis::trajectory::via_point &from, const praxis::trajectory::via_point &to);

praxis::expected<praxis::trajectory::trajectory_sample, praxis::refusal> segment_sample(const cubic_segment &segment, double dt);

}

#endif

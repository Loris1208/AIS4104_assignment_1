#ifndef HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_TIME_SCALING_H
#define HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_TIME_SCALING_H

#include "rigid_motion/alias.h"

#include <praxis/trajectory/time_scaling.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

namespace ais4104::trajectory {

praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> cubic(double t, double duration);

praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> quintic(double t, double duration);

}

#endif

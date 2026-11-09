#ifndef HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_CAPABILITIES_H
#define HPP_GUARD_AIS4104_TRAJECTORY_GENERATION_CAPABILITIES_H

#include <praxis/trajectory/path.h>
#include <praxis/trajectory/trajectory.h>
#include <praxis/trajectory/time_scaling.h>

namespace ais4104::trajectory {

praxis::trajectory::time_scaling_ops time_scalings();

praxis::trajectory::path_ops paths();

praxis::trajectory::trajectory_ops trajectories();

}

#endif

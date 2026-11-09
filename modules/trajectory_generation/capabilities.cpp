#include "trajectory_generation/path.h"
#include "trajectory_generation/capabilities.h"
#include "trajectory_generation/time_scaling.h"
#include "trajectory_generation/via_point_trajectory.h"

#include <praxis/trajectory/capabilities.h>

namespace ais4104::trajectory {

namespace {

constexpr praxis::trajectory::path_ops bound_path{
        .joint_straight_line = &joint_straight_line,
        .screw               = &screw,
        .decoupled           = &decoupled,
};

}

praxis::trajectory::time_scaling_ops time_scalings()
{
    praxis::trajectory::time_scaling_ops bound = praxis::trajectory::baseline().time_scaling;
    bound.cubic                                = &cubic;
    bound.quintic                              = &quintic;

    return bound;
}

praxis::trajectory::path_ops paths()
{
    return bound_path;
}

praxis::trajectory::trajectory_ops trajectories()
{
    praxis::trajectory::trajectory_ops bound = praxis::trajectory::baseline().trajectory;
    bound.joint_space_via_points             = &joint_space_via_points;

    return bound;
}

}

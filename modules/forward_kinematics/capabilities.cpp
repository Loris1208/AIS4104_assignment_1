#include "forward_kinematics/kinematics.h"
#include "forward_kinematics/capabilities.h"

namespace ais4104::manipulator {

namespace {

constexpr praxis::manipulator::forward_kinematics_ops bound_forward{
        .forward_kinematics      = &forward_kinematics,
        .body_forward_kinematics = &body_forward_kinematics,
        .body_screws_from_space  = &body_screws_from_space,
};

}

praxis::manipulator::forward_kinematics_ops forward_maps()
{
    return bound_forward;
}

}

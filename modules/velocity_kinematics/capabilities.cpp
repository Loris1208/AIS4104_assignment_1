#include "velocity_kinematics/jacobian.h"
#include "velocity_kinematics/capabilities.h"

namespace ais4104::manipulator {

namespace {

constexpr praxis::manipulator::differential_kinematics_ops bound_differential{
        .space_jacobian = &space_jacobian,
        .body_jacobian  = &body_jacobian,
};

}

praxis::manipulator::differential_kinematics_ops jacobians()
{
    return bound_differential;
}

}

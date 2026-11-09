#include "arm.h"
#include "shapes.h"
#include "motions.h"
#include "solving.h"
#include "functions.h"
#include "minimization.h"
#include "root_finding.h"
#include "arm_slot_report.h"

#include "trajectory_generation/capabilities.h"

#include <praxis/manipulator/capabilities.h>

#include <praxis/trajectory/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/optimization/capabilities.h>

namespace ais4104 {

praxis::manipulator::capabilities offered_arm()
{
    return praxis::manipulator::baseline();
}

tool_offset_source offered_tool_offsets()
{
    return tool_offset_source::hand_derived;
}

praxis::rigid_motion::capabilities offered_motions()
{
    return praxis::rigid_motion::baseline();
}

praxis::trajectory::capabilities offered_shapes()
{
    praxis::trajectory::capabilities composed = praxis::trajectory::baseline();
    composed.time_scaling                     = trajectory::time_scalings();
    composed.path                             = trajectory::paths();
    composed.trajectory                       = trajectory::trajectories();

    return composed;
}

praxis::optimization::capabilities offered_solving()
{
    return praxis::optimization::baseline();
}

offered_function_forms offered_functions()
{
    return {.root = {.curves = {cubic_curve(), arctangent_curve()}, .surfaces = {}}, .minimum = {.curves = {parabola_curve(), quartic_curve()}, .surfaces = {}}};
}

owned_slots offered_owned_slots()
{
    owned_slots owned{};
    owned.time_scaling.set(praxis::trajectory::time_scaling_slot::cubic);
    owned.time_scaling.set(praxis::trajectory::time_scaling_slot::quintic);
    owned.path = every_slot<praxis::trajectory::path_slot>();
    owned.trajectory.set(praxis::trajectory::trajectory_slot::joint_space_via_points);

    return owned;
}

}

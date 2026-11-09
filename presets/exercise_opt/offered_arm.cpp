#include "arm.h"
#include "shapes.h"
#include "motions.h"
#include "solving.h"
#include "functions.h"
#include "minimization.h"
#include "root_finding.h"
#include "arm_slot_report.h"

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
    return tool_offset_source::document;
}

praxis::rigid_motion::capabilities offered_motions()
{
    return praxis::rigid_motion::baseline();
}

praxis::trajectory::capabilities offered_shapes()
{
    return praxis::trajectory::baseline();
}

praxis::optimization::capabilities offered_solving()
{
    return praxis::optimization::baseline();
}

offered_function_forms offered_functions()
{
    return {.root = {.curves = root_finding_curves(), .surfaces = root_finding_surfaces()}, .minimum = {.curves = minimization_curves(), .surfaces = minimization_surfaces()}};
}

owned_slots offered_owned_slots()
{
    owned_slots owned{};
    owned.robot = every_slot<praxis::manipulator::robot_slot>();
    owned.time_scaling.set(praxis::trajectory::time_scaling_slot::cubic);
    owned.time_scaling.set(praxis::trajectory::time_scaling_slot::quintic);
    owned.path = every_slot<praxis::trajectory::path_slot>();
    owned.trajectory.set(praxis::trajectory::trajectory_slot::joint_space_via_points);
    owned.solver = every_slot<praxis::optimization::solver_slot>();
    owned.minimizer.set(praxis::optimization::minimizer_slot::gradient_descent);

    return owned;
}

}

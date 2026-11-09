#include "arm.h"
#include "shapes.h"
#include "motions.h"
#include "solving.h"
#include "functions.h"
#include "minimization.h"
#include "root_finding.h"
#include "arm_slot_report.h"

#include "optimization/capabilities.h"
#include "inverse_kinematics/capabilities.h"

#include <praxis/manipulator/capabilities.h>
#include <praxis/manipulator/solver_adapters.h>

#include <praxis/trajectory/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/optimization/capabilities.h>

namespace ais4104 {

praxis::manipulator::capabilities offered_arm()
{
    praxis::manipulator::capabilities composed = praxis::manipulator::baseline();
    composed.ik.inverse_kinematics             = &praxis::manipulator::body_inverse_kinematics;
    composed.robot                             = manipulator::end_effector_frames();

    return composed;
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
    return praxis::trajectory::baseline();
}

praxis::optimization::capabilities offered_solving()
{
    praxis::optimization::capabilities composed = praxis::optimization::baseline();
    composed.solver                             = optimization::solvers();
    composed.minimizer                          = optimization::minimizers();

    return composed;
}

offered_function_forms offered_functions()
{
    return {.root = {.curves = {cubic_curve(), arctangent_curve()}, .surfaces = {}}, .minimum = {.curves = {parabola_curve(), quartic_curve()}, .surfaces = {}}};
}

owned_slots offered_owned_slots()
{
    owned_slots owned{};
    owned.robot  = every_slot<praxis::manipulator::robot_slot>();
    owned.solver = every_slot<praxis::optimization::solver_slot>();
    owned.minimizer.set(praxis::optimization::minimizer_slot::gradient_descent);

    return owned;
}

}

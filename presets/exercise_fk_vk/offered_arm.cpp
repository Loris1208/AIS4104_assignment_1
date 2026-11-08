#include "arm.h"
#include "shapes.h"
#include "motions.h"
#include "solving.h"
#include "functions.h"
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
    return {};
}

owned_slots offered_owned_slots()
{
    owned_slots owned{};
    owned.fk = every_slot<praxis::manipulator::forward_kinematics_slot>();
    owned.dk = every_slot<praxis::manipulator::differential_kinematics_slot>();

    return owned;
}

}

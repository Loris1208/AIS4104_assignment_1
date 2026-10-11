#include "arm.h"
#include "motions.h"

#include "rigid_motion/capabilities.h"
#include "forward_kinematics/capabilities.h"
#include "velocity_kinematics/capabilities.h"

#include <praxis/manipulator/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

namespace ais4104 {

praxis::manipulator::capabilities offered_arm()
{
    praxis::manipulator::capabilities composed = praxis::manipulator::baseline();
    composed.fk                                = manipulator::forward_maps();
    composed.dk                                = manipulator::jacobians();

    return composed;
}

tool_offset_source offered_tool_offsets()
{
    return tool_offset_source::hand_derived;
}

praxis::rigid_motion::capabilities offered_motions()
{
    return rigid_motion::motions();
}

}

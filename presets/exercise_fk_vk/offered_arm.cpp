#include "arm.h"
#include "motions.h"

#include <praxis/manipulator/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

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

}

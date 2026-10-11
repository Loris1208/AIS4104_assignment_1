#ifndef HPP_GUARD_AIS4104_APPLICATION_HAND_DERIVED_H
#define HPP_GUARD_AIS4104_APPLICATION_HAND_DERIVED_H

#include "arm.h"

#include <praxis/presets/arm.h>

#include <praxis/manipulator/compose_arm.h>

#include <praxis/rigid_motion/frame.h>

#include <praxis/config/document.h>

namespace ais4104 {

// Where offsets reads hand-derived tool offsets and the document names one under hand_derived/tool,
// the tool's kinematics transform is that function's answer, rotation and translation, in the
// document's euler order. An answer that is zero, not finite, whose bottom row is not (0, 0, 0, 1),
// or whose rotation block is not a rotation stands the tool center point at the end frame, aligned
// with it.
praxis::presets::arm_scenario with_hand_derived_tool(praxis::presets::arm_scenario machine, const praxis::config::document &values, const praxis::rigid_motion::frame_ops &frames,
                                                     tool_offset_source offsets);

// Where with_hand_derived_tool read the tool center point, the Tool window writes its model, view and
// graphics transform under praxis's tool path, and never its kinematics transform, which that function
// answers on every opening.
praxis::manipulator::arm_composition with_hand_derived_tool_window(praxis::manipulator::arm_composition composed, const praxis::presets::arm_scenario &machine,
                                                                   const praxis::config::document &values, tool_offset_source offsets);

}

#endif

#ifndef HPP_GUARD_AIS4104_APPLICATION_ARM_H
#define HPP_GUARD_AIS4104_APPLICATION_ARM_H

#include <praxis/manipulator/capabilities.h>

#include <cstdint>

namespace ais4104 {

enum class tool_offset_source : std::uint8_t
{
    document,
    hand_derived
};

praxis::manipulator::capabilities offered_arm();

// Where this executable reads the tool center point of a document naming a hand-derived tool offset.
tool_offset_source offered_tool_offsets();

}

#endif

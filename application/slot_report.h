#ifndef HPP_GUARD_AIS4104_APPLICATION_SLOT_REPORT_H
#define HPP_GUARD_AIS4104_APPLICATION_SLOT_REPORT_H

#include <praxis/rigid_motion/capabilities.h>

#include <vector>
#include <cstddef>
#include <string_view>

namespace ais4104 {

enum class slot_state
{
    written_and_correct,
    unwritten,
    written_and_disagrees
};

struct classified_slot
{
    slot_state state;
    std::string_view name;
};

std::vector<classified_slot> classify_slots(const praxis::rigid_motion::capabilities &motions, std::size_t cases);

void report_slot_state(const praxis::rigid_motion::capabilities &motions);

}

#endif

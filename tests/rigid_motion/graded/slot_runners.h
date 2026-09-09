#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_GRADED_SLOT_RUNNERS_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_GRADED_SLOT_RUNNERS_H

#include "graded_tasks.h"

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/slot_evaluation.h>

#include <cstddef>
#include <string_view>

namespace ais4104 {

// One entry per graded task: the assignment's own identifier, and how that slot's rows are answered
// and judged.
struct graded_rows
{
    std::string_view task;
    std::size_t (*run)(const praxis::rigid_motion::capabilities &, const graded_task &, const praxis::evaluation::tolerance_pair &);
};

}

#endif

#include "graded_tasks.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/evaluation.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/slot_evaluation.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace {

constexpr std::size_t rigid_motion_slots = 29;

// Three tasks, whose subtasks are lettered unbroken from b because a) of each asks the student to
// verify the initial test suite and owns no slot.
constexpr std::array<std::size_t, 3> subtasks_per_task{9, 5, 15};

static_assert(ais4104::graded_tasks.size() == rigid_motion_slots);

std::vector<std::string> assignment_identifiers()
{
    std::vector<std::string> named;
    for(std::size_t task = 0; task < subtasks_per_task.size(); ++task)
        for(std::size_t subtask = 0; subtask < subtasks_per_task[task]; ++subtask)
            named.push_back(std::to_string(task + 1) + static_cast<char>('b' + subtask));

    return named;
}

void require_slot_matches_table(const praxis::evaluation::slot_evaluation &slot, std::size_t index)
{
    REQUIRE(index < ais4104::graded_tasks.size());

    REQUIRE(ais4104::graded_tasks[index].slot == slot.name);
}

void require_identifiers_match_the_assignment()
{
    const std::vector<std::string> named = assignment_identifiers();
    REQUIRE(named.size() == ais4104::graded_tasks.size());

    for(const std::string &identifier : named)
    {
        INFO(identifier);
        REQUIRE(ais4104::graded_task_of(identifier) != nullptr);
    }

    for(const ais4104::graded_task &entry : ais4104::graded_tasks)
    {
        INFO(entry.task);
        REQUIRE(std::count(named.begin(), named.end(), std::string{entry.task}) == 1);
        REQUIRE(ais4104::rows_naming(ais4104::graded_tasks, entry.task) == 1);
    }
}

}

TEST_CASE("the graded task table names praxis's rigid-motion slots in praxis's own order", "[rigid_motion][grading]")
{
    const praxis::rigid_motion::capabilities composed  = ais4104::rigid_motion::motions();
    const praxis::rigid_motion::capabilities reference = praxis::rigid_motion::baseline();

    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(composed, reference);

    std::size_t index = 0;
    for(const praxis::evaluation::evaluation_view &view : compared)
        for(const praxis::evaluation::slot_evaluation &slot : view.slots())
        {
            require_slot_matches_table(slot, index);
            ++index;
        }

    REQUIRE(index == ais4104::graded_tasks.size());
    require_identifiers_match_the_assignment();
}

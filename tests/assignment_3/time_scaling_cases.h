#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_3_TIME_SCALING_CASES_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_3_TIME_SCALING_CASES_H

#include "suite_rows.h"

#include <array>
#include <cstddef>
#include <algorithm>

namespace ais4104 {

// The slot is 0 for the cubic and 1 for the quintic; duration and time in seconds.
struct time_scaling_case
{
    std::size_t slot;
    double duration;
    double time;
};

constexpr std::array<time_scaling_case, 40> time_scaling_cases{
        time_scaling_case{.slot = 0, .duration = 0.25, .time = 0.0},   time_scaling_case{.slot = 0, .duration = 0.25, .time = 0.0625},
        time_scaling_case{.slot = 0, .duration = 0.25, .time = 0.125}, time_scaling_case{.slot = 0, .duration = 0.25, .time = 0.1875},
        time_scaling_case{.slot = 0, .duration = 0.25, .time = 0.25},  time_scaling_case{.slot = 0, .duration = 1.0, .time = 0.0},
        time_scaling_case{.slot = 0, .duration = 1.0, .time = 0.25},   time_scaling_case{.slot = 0, .duration = 1.0, .time = 0.5},
        time_scaling_case{.slot = 0, .duration = 1.0, .time = 0.75},   time_scaling_case{.slot = 0, .duration = 1.0, .time = 1.0},
        time_scaling_case{.slot = 0, .duration = 5.0, .time = 0.0},    time_scaling_case{.slot = 0, .duration = 5.0, .time = 1.25},
        time_scaling_case{.slot = 0, .duration = 5.0, .time = 2.5},    time_scaling_case{.slot = 0, .duration = 5.0, .time = 3.75},
        time_scaling_case{.slot = 0, .duration = 5.0, .time = 5.0},    time_scaling_case{.slot = 0, .duration = 20.0, .time = 0.0},
        time_scaling_case{.slot = 0, .duration = 20.0, .time = 5.0},   time_scaling_case{.slot = 0, .duration = 20.0, .time = 10.0},
        time_scaling_case{.slot = 0, .duration = 20.0, .time = 15.0},  time_scaling_case{.slot = 0, .duration = 20.0, .time = 20.0},
        time_scaling_case{.slot = 1, .duration = 0.25, .time = 0.0},   time_scaling_case{.slot = 1, .duration = 0.25, .time = 0.0625},
        time_scaling_case{.slot = 1, .duration = 0.25, .time = 0.125}, time_scaling_case{.slot = 1, .duration = 0.25, .time = 0.1875},
        time_scaling_case{.slot = 1, .duration = 0.25, .time = 0.25},  time_scaling_case{.slot = 1, .duration = 1.0, .time = 0.0},
        time_scaling_case{.slot = 1, .duration = 1.0, .time = 0.25},   time_scaling_case{.slot = 1, .duration = 1.0, .time = 0.5},
        time_scaling_case{.slot = 1, .duration = 1.0, .time = 0.75},   time_scaling_case{.slot = 1, .duration = 1.0, .time = 1.0},
        time_scaling_case{.slot = 1, .duration = 5.0, .time = 0.0},    time_scaling_case{.slot = 1, .duration = 5.0, .time = 1.25},
        time_scaling_case{.slot = 1, .duration = 5.0, .time = 2.5},    time_scaling_case{.slot = 1, .duration = 5.0, .time = 3.75},
        time_scaling_case{.slot = 1, .duration = 5.0, .time = 5.0},    time_scaling_case{.slot = 1, .duration = 20.0, .time = 0.0},
        time_scaling_case{.slot = 1, .duration = 20.0, .time = 5.0},   time_scaling_case{.slot = 1, .duration = 20.0, .time = 10.0},
        time_scaling_case{.slot = 1, .duration = 20.0, .time = 15.0},  time_scaling_case{.slot = 1, .duration = 20.0, .time = 20.0},
};

// The greatest entry difference over (s, ds, dds), twice.
constexpr double cubic_magnitude   = 1e-12;
constexpr double cubic_linear      = 1e-12;
constexpr double quintic_magnitude = 1e-11;
constexpr double quintic_linear    = 1e-11;

constexpr pinned_bound cubic_bound{cubic_magnitude, cubic_linear, bound_standing::at_the_platform, bound_standing::at_the_platform};
constexpr pinned_bound quintic_bound{quintic_magnitude, quintic_linear, bound_standing::at_the_platform, bound_standing::at_the_platform};

constexpr bool sampled_at(std::size_t slot, double duration, double time)
{
    return std::ranges::any_of(time_scaling_cases, [&](const time_scaling_case &each) { return each.slot == slot && each.duration == duration && each.time == time; });
}

constexpr bool ends_sampled(const time_scaling_case &each)
{
    return sampled_at(each.slot, each.duration, 0.0) && sampled_at(each.slot, each.duration, each.duration);
}

static_assert(std::ranges::count(time_scaling_cases, std::size_t{0}, &time_scaling_case::slot) == 20);
static_assert(std::ranges::count(time_scaling_cases, std::size_t{1}, &time_scaling_case::slot) == 20);
static_assert(std::ranges::all_of(time_scaling_cases, [](const time_scaling_case &each) { return 0.0 < each.duration; }));
static_assert(std::ranges::all_of(time_scaling_cases, [](const time_scaling_case &each) { return 0.0 <= each.time && each.time <= each.duration; }));
static_assert(std::ranges::all_of(time_scaling_cases, ends_sampled));

}

#endif

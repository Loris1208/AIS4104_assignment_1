#include "suite_rows.h"
#include "time_scaling_cases.h"

#include "trajectory_generation/capabilities.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/trajectory/time_scaling.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <format>
#include <string>
#include <cstddef>
#include <algorithm>

namespace ais4104 {
namespace {

using scaling      = answer<praxis::trajectory::scaling_sample>;
using scaling_slot = scaling (*)(double t, double duration);

const praxis::trajectory::time_scaling_ops &student_scalings()
{
    static const praxis::trajectory::time_scaling_ops scalings = trajectory::time_scalings();

    return scalings;
}

scaling cubic_of(double t, double duration)
{
    return student_scalings().cubic(t, duration);
}

scaling quintic_of(double t, double duration)
{
    return student_scalings().quintic(t, duration);
}

bool finite(const praxis::trajectory::scaling_sample &sample)
{
    return std::isfinite(sample.s) && std::isfinite(sample.ds) && std::isfinite(sample.dds);
}

// Lynch & Park, Modern Robotics, sec. 9.2.2: where the second derivative steps, at t = 0 and t = T,
// either neighboring value is accepted, the reference's or the zero outside the motion.
double second_derivative_difference(const time_scaling_case &each, double held, double expected)
{
    const double difference = std::abs(held - expected);
    if(each.time != 0.0 && each.time != each.duration)
        return difference;

    return std::min(difference, std::abs(held));
}

case_reading scaling_reading(const time_scaling_case &each, const scaling &answered, const praxis::trajectory::scaling_sample &expected, const pinned_bound &bound)
{
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};
    if(!finite(*answered))
        return case_reading{false, " non-finite"};

    const double dds                                 = second_derivative_difference(each, answered->dds, expected.dds);
    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const praxis::evaluation::residual seen = praxis::evaluation::element_wise_residual(Eigen::Vector3d(answered->s, answered->ds, dds), Eigen::Vector3d(expected.s, expected.ds, 0.0));

    return case_reading{seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres, residual_clause(seen, allowed)};
}

void require_reference(const graded_row &row, std::size_t index, bool answered)
{
    INFO(std::format("{} case {}: the reference time scaling refused", row.slot, index));
    REQUIRE(answered);
}

std::size_t run_slot(const graded_row &row, std::size_t slot, scaling_slot student, scaling_slot reference)
{
    std::size_t index = 0;
    for(const time_scaling_case &each : time_scaling_cases)
    {
        if(each.slot != slot)
            continue;

        const scaling expected = reference(each.time, each.duration);
        require_reference(row, index, expected.has_value());
        const case_reading read = scaling_reading(each, student(each.time, each.duration), *expected, row.bound);
        INFO(row_line(row.task, row.slot, "", index, read.clause));
        CHECK(read.passed);
        ++index;
    }

    return index;
}

std::size_t run_cubic(const graded_row &row)
{
    return run_slot(row, 0, &cubic_of, reference_trajectory().time_scaling.cubic);
}

std::size_t run_quintic(const graded_row &row)
{
    return run_slot(row, 1, &quintic_of, reference_trajectory().time_scaling.quintic);
}

constexpr std::array<graded_row, 2> time_scaling_rows{{
        graded_row{"T.4a", "time_scaling.cubic", "time_scaling.cubic", cubic_bound, &run_cubic},
        graded_row{"T.4b", "time_scaling.quintic", "time_scaling.quintic", quintic_bound, &run_quintic},
}};

}
}

TEST_CASE("every graded time scaling answers what the reference answers", "[time_scaling][graded]")
{
    for(const ais4104::graded_row &row : ais4104::time_scaling_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row);
        }
    }
}

#include "suite_rows.h"
#include "via_point_cases.h"

#include "trajectory_generation/via_points.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/trajectory/trajectory.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <format>
#include <vector>
#include <cstddef>

namespace ais4104 {
namespace {

using coefficients = answer<trajectory::cubic_segment>;

bool well_formed(const trajectory::cubic_segment &segment, Eigen::Index width)
{
    const auto fits = [width](const Eigen::VectorXd &coefficient) { return coefficient.size() == width && coefficient.allFinite(); };

    return fits(segment.a0) && fits(segment.a1) && fits(segment.a2) && fits(segment.a3);
}

double condition_residual(const Eigen::VectorXd &held, const Eigen::VectorXd &against)
{
    const Eigen::ArrayXd apart = (held - against).array().abs();
    const Eigen::ArrayXd scale = held.array().abs().max(against.array().abs()).max(1.0);

    return (apart / scale).maxCoeff<Eigen::PropagateNaN>();
}

// The four conditions of Lynch & Park, Modern Robotics, sec. 9.3: the segment from via point j to j + 1 starts at theta_j with
// velocity theta_dot_j and, dt = T_{j+1} - T_j later, is at theta_{j+1} with velocity theta_dot_{j+1}. Each is read per joint as
// the difference over the larger of one and either side's magnitude.
praxis::evaluation::residual conditions_residual(const trajectory::cubic_segment &segment, const praxis::trajectory::via_point &from, const praxis::trajectory::via_point &to)
{
    const double dt                  = to.time - from.time;
    const Eigen::VectorXd end        = segment.a0 + segment.a1 * dt + segment.a2 * dt * dt + segment.a3 * dt * dt * dt;
    const Eigen::VectorXd end_moving = segment.a1 + 2.0 * segment.a2 * dt + 3.0 * segment.a3 * dt * dt;
    const Eigen::Vector4d parts(condition_residual(segment.a0, from.position), condition_residual(segment.a1, from.velocity), condition_residual(end, to.position),
                                condition_residual(end_moving, to.velocity));

    return praxis::evaluation::element_wise_residual(parts, Eigen::Vector4d::Zero());
}

case_reading segment_reading(const coefficients &answered, const praxis::trajectory::via_point &from, const praxis::trajectory::via_point &to, const pinned_bound &bound)
{
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};
    if(!well_formed(*answered, from.position.size()))
        return case_reading{false, " non-finite or of another width"};

    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const praxis::evaluation::residual seen          = conditions_residual(*answered, from, to);

    return case_reading{seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres, residual_clause(seen, allowed)};
}

std::size_t run_segments(const graded_row &row)
{
    std::size_t read = 0;
    for(std::size_t run = 0; run < via_point_runs.size(); ++run)
    {
        const std::vector<praxis::trajectory::via_point> points = via_points_of(via_point_runs[run]);
        for(std::size_t j = 0; j + 1 < points.size(); ++j, ++read)
        {
            const case_reading reading = segment_reading(trajectory::segment_coefficients(points[j], points[j + 1]), points[j], points[j + 1], row.bound);
            INFO(row_line(row.task, row.slot, std::format("run {} segment {}", run, j), read, reading.clause));
            CHECK(reading.passed);
        }
    }

    return read;
}

constexpr graded_row via_segment_row{"T.5a", "via_points.segment_coefficients", "trajectory.joint_space_via_points", via_segment_bound, &run_segments};

}
}

TEST_CASE("every graded via-point segment meets its two via points", "[via_points][graded]")
{
    DYNAMIC_SECTION(std::format("task {} - {}", ais4104::via_segment_row.task, ais4104::via_segment_row.slot))
    {
        ais4104::check_row(ais4104::via_segment_row);
    }
}

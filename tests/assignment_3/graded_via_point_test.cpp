#include "suite_rows.h"
#include "via_point_cases.h"

#include "trajectory_generation/capabilities.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/trajectory/trajectory.h>
#include <praxis/trajectory/time_scaling.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <string_view>

namespace ais4104 {
namespace {

using generated = answer<std::unique_ptr<praxis::trajectory::trajectory_generator>>;
using sampled   = answer<praxis::trajectory::trajectory_sample>;

const praxis::trajectory::trajectory_ops &student_trajectories()
{
    static const praxis::trajectory::trajectory_ops trajectories = trajectory::trajectories();

    return trajectories;
}

bool comparable(const praxis::trajectory::trajectory_sample &held, const praxis::trajectory::trajectory_sample &expected)
{
    const Eigen::Index width = expected.position.size();
    return held.position.size() == width && held.velocity.size() == width && held.acceleration.size() == width && held.position.allFinite() && held.velocity.allFinite() &&
            held.acceleration.allFinite();
}

// As praxis judges two runs: each acceleration over the larger of one and either side's magnitude, and at the first and last sample,
// where the motion's acceleration steps to rest, either side at rest is accepted.
double acceleration_difference(const praxis::trajectory::configuration &held, const praxis::trajectory::configuration &expected, bool at_an_end)
{
    double worst = 0.0;
    for(Eigen::Index i = 0; i < held.size(); ++i)
    {
        const double apart = std::abs(held[i] - expected[i]);
        const double owed  = at_an_end ? std::min({apart, std::abs(held[i]), std::abs(expected[i])}) : apart;
        worst              = std::max(worst, owed / std::max({1.0, std::abs(held[i]), std::abs(expected[i])}));
    }

    return worst;
}

case_reading sample_reading(const sampled &answered, const praxis::trajectory::trajectory_sample &expected, bool at_an_end, const pinned_bound &bound)
{
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};
    if(!comparable(*answered, expected))
        return case_reading{false, " non-finite or of another width"};

    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const Eigen::Vector3d parts(praxis::evaluation::element_wise_residual(answered->position, expected.position).magnitude,
                                praxis::evaluation::element_wise_residual(answered->velocity, expected.velocity).magnitude,
                                acceleration_difference(answered->acceleration, expected.acceleration, at_an_end));
    const praxis::evaluation::residual seen = praxis::evaluation::element_wise_residual(parts, Eigen::Vector3d::Zero());

    return case_reading{seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres, residual_clause(seen, allowed)};
}

void require_reference(const graded_row &row, std::size_t run, bool answered)
{
    INFO(std::format("{} run {}: the reference refused", row.slot, run));
    REQUIRE(answered);
}

bool generated_by_student(const graded_row &row, std::string_view subject, const generated &student)
{
    const bool answered = student.has_value() && *student != nullptr;
    INFO(row_line(row.task, row.slot, subject, 0, student ? std::string(answered ? "" : " answered no generator") : refused_clause(student.error())));
    CHECK(answered);

    return answered;
}

void check_duration(const graded_row &row, std::size_t run, double held, double expected)
{
    INFO(std::format("task {} - {} run {}: duration {:.17e} against {:.17e}", row.task, row.slot, run, held, expected));
    CHECK(std::abs(held - expected) <= duration_tolerance * std::max({1.0, std::abs(held), std::abs(expected)}));
}

void check_sample(const graded_row &row, std::string_view subject, const sample_point &at, const sampled &held, const praxis::trajectory::trajectory_sample &expected)
{
    const case_reading read = sample_reading(held, expected, at.at_an_end, row.bound);
    INFO(row_line(row.task, row.slot, subject, at.index, read.clause));
    CHECK(read.passed);
}

std::size_t check_run(const graded_row &row, std::size_t run)
{
    const std::vector<praxis::trajectory::via_point> points = via_points_of(via_point_runs[run]);
    const generated reference                               = reference_trajectory().trajectory.joint_space_via_points(points);
    require_reference(row, run, reference.has_value());
    const generated student = student_trajectories().joint_space_via_points(points);
    if(!generated_by_student(row, std::format("run {} segment 0", run), student))
        return 1;

    check_duration(row, run, (*student)->duration(), (*reference)->duration());
    const std::vector<sample_point> samples = sample_points(via_point_runs[run]);
    for(const sample_point &at : samples)
    {
        const sampled expected = (*reference)->sample(at.time);
        require_reference(row, run, expected.has_value());
        check_sample(row, std::format("run {} segment {}", run, at.segment), at, (*student)->sample(at.time), *expected);
    }

    return samples.size();
}

// Lynch & Park, Modern Robotics, sec. 9.2.1 and 9.3: two via points at rest are joined by the rest-to-rest cubic,
// theta_s + s(t)(theta_e - theta_s).
sampled rest_to_rest(const std::vector<praxis::trajectory::via_point> &points, double t)
{
    const answer<praxis::trajectory::scaling_sample> scaled = reference_trajectory().time_scaling.cubic(t, points.back().time);
    if(!scaled)
        return praxis::unexpected(scaled.error());

    const praxis::trajectory::configuration apart = points.back().position - points.front().position;
    return praxis::trajectory::trajectory_sample{points.front().position + scaled->s * apart, scaled->ds * apart, scaled->dds * apart};
}

std::size_t check_at_rest(const graded_row &row, std::size_t run)
{
    const std::vector<praxis::trajectory::via_point> points = via_points_of(via_point_runs[run]);
    const std::string subject                               = std::format("at rest run {}", run);
    const generated student                                 = student_trajectories().joint_space_via_points(points);
    if(!generated_by_student(row, subject, student))
        return 1;

    check_duration(row, run, (*student)->duration(), points.back().time);
    const std::vector<sample_point> samples = sample_points(via_point_runs[run]);
    for(const sample_point &at : samples)
    {
        const sampled expected = rest_to_rest(points, at.time);
        require_reference(row, run, expected.has_value());
        check_sample(row, subject, at, (*student)->sample(at.time), *expected);
    }

    return samples.size();
}

std::size_t run_runs(const graded_row &row)
{
    std::size_t read = 0;
    for(std::size_t run = 0; run < via_point_runs.size(); ++run)
        read += check_run(row, run);

    return read;
}

std::size_t run_at_rest(const graded_row &row)
{
    std::size_t read = 0;
    for(const std::size_t run : at_rest_runs)
        read += check_at_rest(row, run);

    return read;
}

constexpr graded_row via_point_row{"T.5b", "via_points.segment_sample", "trajectory.joint_space_via_points", via_point_bound, &run_runs};
constexpr graded_row at_rest_row{"T.5b", "via_points.segment_sample", "trajectory.joint_space_via_points", at_rest_bound, &run_at_rest};

}
}

TEST_CASE("every graded via-point run answers what the reference answers", "[via_points][graded]")
{
    DYNAMIC_SECTION(std::format("task {} - {}", ais4104::via_point_row.task, ais4104::via_point_row.slot))
    {
        ais4104::check_row(ais4104::via_point_row);
    }
}

TEST_CASE("a two-point via-point run at rest is the rest-to-rest cubic", "[via_points][graded]")
{
    DYNAMIC_SECTION(std::format("task {} - {}", ais4104::at_rest_row.task, ais4104::at_rest_row.slot))
    {
        ais4104::check_row(ais4104::at_rest_row);
    }
}

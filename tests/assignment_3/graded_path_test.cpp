#include "path_cases.h"
#include "suite_rows.h"
#include "robot_cases.h"
#include "inverse_kinematics_cases.h"

#include "trajectory_generation/capabilities.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/trajectory/path.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <cstddef>
#include <numbers>

namespace ais4104 {
namespace {

using pose_path = answer<Eigen::Matrix4d> (*)(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                              const Eigen::Matrix4d &end, double s);

const praxis::trajectory::path_ops &student_paths()
{
    static const praxis::trajectory::path_ops paths = trajectory::paths();

    return paths;
}

answer<Eigen::VectorXd> joint_straight_line_of(const Eigen::VectorXd &start, const Eigen::VectorXd &end, double s)
{
    return student_paths().joint_straight_line(start, end, s);
}

answer<Eigen::Matrix4d> screw_path_of(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                      const Eigen::Matrix4d &end, double s)
{
    return student_paths().screw(motion, frames, start, end, s);
}

answer<Eigen::Matrix4d> decoupled_path_of(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                          const Eigen::Matrix4d &end, double s)
{
    return student_paths().decoupled(motion, frames, start, end, s);
}

case_reading judged(const praxis::evaluation::residual &seen, const pinned_bound &bound)
{
    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);

    return case_reading{seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres, residual_clause(seen, allowed)};
}

case_reading joint_reading(const answer<Eigen::VectorXd> &answered, const Eigen::VectorXd &expected, const pinned_bound &bound)
{
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};
    if(answered->size() != expected.size())
        return case_reading{false, std::format(" answered {} joints for a path of {}", answered->size(), expected.size())};

    return judged(praxis::evaluation::element_wise_residual(*answered, expected), bound);
}

case_reading pose_reading(const answer<Eigen::Matrix4d> &answered, const Eigen::Matrix4d &expected, const pinned_bound &bound)
{
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};

    return judged(praxis::evaluation::pose_residual(*answered, expected), bound);
}

void require_reference(const graded_row &row, std::size_t index, bool answered)
{
    INFO(std::format("{} case {}: the reference path refused", row.slot, index));
    REQUIRE(answered);
}

void check_reading(const graded_row &row, std::size_t index, const case_reading &read)
{
    INFO(row_line(row.task, row.slot, "", index, read.clause));
    CHECK(read.passed);
}

void require_clear_of_turns(const graded_row &row, std::size_t pair, const Eigen::Matrix4d &start, const Eigen::Matrix4d &end)
{
    const double angle   = praxis::evaluation::pose_residual(start, end).magnitude;
    const double highest = std::numbers::pi - rotation_margin_pi;
    const bool clear     = rotation_margin_zero <= angle && angle <= highest;

    INFO(std::format("{} pair {}: the relative rotation {:.6e} rad, required inside [{:.6e}, {:.6e}]", row.slot, pair, angle, rotation_margin_zero, highest));
    REQUIRE(clear);
}

void check_joint_pair(const graded_row &row, std::size_t pair)
{
    const std::size_t joints    = frozen_chains[joint_pairs[pair].subject].joints;
    const Eigen::VectorXd start = leading_entries(joint_pairs[pair].start, joints);
    const Eigen::VectorXd end   = leading_entries(joint_pairs[pair].end, joints);
    for(std::size_t sample = 0; sample < path_samples.size(); ++sample)
    {
        const std::size_t index                = pair * path_samples.size() + sample;
        const answer<Eigen::VectorXd> expected = reference_trajectory().path.joint_straight_line(start, end, path_samples[sample]);
        require_reference(row, index, expected.has_value());
        check_reading(row, index, joint_reading(joint_straight_line_of(start, end, path_samples[sample]), *expected, row.bound));
    }
}

void check_pose_pair(const graded_row &row, std::size_t pair, pose_path student, pose_path reference)
{
    const Eigen::Matrix4d start = matrix_of(pose_pairs[pair].start);
    const Eigen::Matrix4d end   = matrix_of(pose_pairs[pair].end);
    require_clear_of_turns(row, pair, start, end);
    for(std::size_t sample = 0; sample < path_samples.size(); ++sample)
    {
        const std::size_t index                = pair * path_samples.size() + sample;
        const answer<Eigen::Matrix4d> expected = reference(reference_motions().screw, reference_motions().frame, start, end, path_samples[sample]);
        require_reference(row, index, expected.has_value());
        check_reading(row, index, pose_reading(student(reference_motions().screw, reference_motions().frame, start, end, path_samples[sample]), *expected, row.bound));
    }
}

std::size_t run_joint_straight_line(const graded_row &row)
{
    for(std::size_t pair = 0; pair < joint_pairs.size(); ++pair)
        check_joint_pair(row, pair);

    return joint_pairs.size() * path_samples.size();
}

std::size_t run_screw(const graded_row &row)
{
    for(std::size_t pair = 0; pair < pose_pairs.size(); ++pair)
        check_pose_pair(row, pair, &screw_path_of, reference_trajectory().path.screw);

    return pose_pairs.size() * path_samples.size();
}

std::size_t run_decoupled(const graded_row &row)
{
    for(std::size_t pair = 0; pair < pose_pairs.size(); ++pair)
        check_pose_pair(row, pair, &decoupled_path_of, reference_trajectory().path.decoupled);

    return pose_pairs.size() * path_samples.size();
}

constexpr std::array<graded_row, 3> path_rows{{
        graded_row{"T.3a", "path.joint_straight_line", "path.joint_straight_line", joint_straight_line_bound, &run_joint_straight_line},
        graded_row{"T.3b", "path.screw", "path.screw", screw_bound, &run_screw},
        graded_row{"T.3c", "path.decoupled", "path.decoupled", decoupled_bound, &run_decoupled},
}};

}
}

TEST_CASE("every graded path answers what the reference answers", "[path][graded]")
{
    for(const ais4104::graded_row &row : ais4104::path_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row);
        }
    }
}

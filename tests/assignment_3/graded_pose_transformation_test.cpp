#include "suite_rows.h"
#include "robot_cases.h"

#include "inverse_kinematics/capabilities.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/manipulator/robot.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <cstddef>

namespace ais4104 {
namespace {

using pose_transformation_reading = case_reading (*)(const pose_transformation_case &each, const pinned_bound &bound);

const praxis::manipulator::robot_ops &student_frames()
{
    static const praxis::manipulator::robot_ops frames = manipulator::end_effector_frames();

    return frames;
}

answer<Eigen::Matrix4d> tool_pose_from_flange_pose_of(const Eigen::Matrix4d &flange, const Eigen::Matrix4d &offset)
{
    return student_frames().tool_pose_from_flange_pose(flange, offset);
}

answer<Eigen::Matrix4d> flange_pose_from_tool_pose_of(const Eigen::Matrix4d &tool, const Eigen::Matrix4d &offset)
{
    return student_frames().flange_pose_from_tool_pose(reference_motions().frame, tool, offset);
}

answer<Eigen::Vector3d> position_from_pose_of(const Eigen::Matrix4d &pose)
{
    return student_frames().position_from_pose(pose);
}

answer<Eigen::Matrix3d> orientation_from_pose_of(const Eigen::Matrix4d &pose)
{
    return student_frames().orientation_from_pose(pose);
}

template<typename T>
T reference_answer(const answer<T> &answered)
{
    REQUIRE(answered.has_value());

    return *answered;
}

case_reading judged(const praxis::evaluation::residual &seen, const pinned_bound &bound)
{
    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);

    return case_reading{seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres, residual_clause(seen, allowed)};
}

case_reading tool_pose_reading(const pose_transformation_case &each, const pinned_bound &bound)
{
    const Eigen::Matrix4d flange           = matrix_of(each.flange);
    const Eigen::Matrix4d offset           = matrix_of(each.offset);
    const Eigen::Matrix4d expected         = reference_answer(reference_manipulator().robot.tool_pose_from_flange_pose(flange, offset));
    const answer<Eigen::Matrix4d> answered = tool_pose_from_flange_pose_of(flange, offset);
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};

    return judged(praxis::evaluation::pose_residual(*answered, expected), bound);
}

case_reading flange_pose_reading(const pose_transformation_case &each, const pinned_bound &bound)
{
    const Eigen::Matrix4d tool             = matrix_of(each.tool);
    const Eigen::Matrix4d offset           = matrix_of(each.offset);
    const Eigen::Matrix4d expected         = reference_answer(reference_manipulator().robot.flange_pose_from_tool_pose(reference_motions().frame, tool, offset));
    const answer<Eigen::Matrix4d> answered = flange_pose_from_tool_pose_of(tool, offset);
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};

    return judged(praxis::evaluation::pose_residual(*answered, expected), bound);
}

case_reading position_reading(const pose_transformation_case &each, const pinned_bound &bound)
{
    const Eigen::Matrix4d flange           = matrix_of(each.flange);
    const Eigen::Vector3d expected         = reference_answer(reference_manipulator().robot.position_from_pose(flange));
    const answer<Eigen::Vector3d> answered = position_from_pose_of(flange);
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};

    return judged(praxis::evaluation::element_wise_residual(*answered, expected), bound);
}

case_reading orientation_reading(const pose_transformation_case &each, const pinned_bound &bound)
{
    const Eigen::Matrix4d flange           = matrix_of(each.flange);
    const Eigen::Matrix3d expected         = reference_answer(reference_manipulator().robot.orientation_from_pose(flange));
    const answer<Eigen::Matrix3d> answered = orientation_from_pose_of(flange);
    if(!answered)
        return case_reading{false, refused_clause(answered.error())};

    return judged(praxis::evaluation::geodesic_residual(*answered, expected), bound);
}

case_reading read_case(const graded_row &row, std::size_t index, pose_transformation_reading reading_of)
{
    INFO(std::format("{} case {}: the reference {} refused", row.slot, index, row.published));

    return reading_of(pose_transformation_cases[index], row.bound);
}

std::size_t run_cases(const graded_row &row, pose_transformation_reading reading_of)
{
    for(std::size_t index = 0; index < pose_transformation_cases.size(); ++index)
    {
        const case_reading verdict = read_case(row, index, reading_of);
        INFO(row_line(row.task, row.slot, "", index, verdict.clause));
        CHECK(verdict.passed);
    }
    return pose_transformation_cases.size();
}

std::size_t run_tool_pose_from_flange_pose(const graded_row &row)
{
    return run_cases(row, &tool_pose_reading);
}

std::size_t run_flange_pose_from_tool_pose(const graded_row &row)
{
    return run_cases(row, &flange_pose_reading);
}

std::size_t run_position_from_pose(const graded_row &row)
{
    return run_cases(row, &position_reading);
}

std::size_t run_orientation_from_pose(const graded_row &row)
{
    return run_cases(row, &orientation_reading);
}

constexpr std::array<graded_row, 4> pose_transformation_rows{{
        graded_row{"T.2a", "end_effector_frames.tool_pose_from_flange_pose", "robot.tool_pose_from_flange_pose", tool_pose_from_flange_pose_bound, &run_tool_pose_from_flange_pose},
        graded_row{"T.2b", "end_effector_frames.flange_pose_from_tool_pose", "robot.flange_pose_from_tool_pose", flange_pose_from_tool_pose_bound, &run_flange_pose_from_tool_pose},
        graded_row{"T.2c", "end_effector_frames.position_from_pose", "robot.position_from_pose", position_from_pose_bound, &run_position_from_pose},
        graded_row{"T.2c", "end_effector_frames.orientation_from_pose", "robot.orientation_from_pose", orientation_from_pose_bound, &run_orientation_from_pose},
}};

}
}

TEST_CASE("every graded pose transformation answers what the reference answers", "[robot][graded]")
{
    for(const ais4104::graded_row &row : ais4104::pose_transformation_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row);
        }
    }
}

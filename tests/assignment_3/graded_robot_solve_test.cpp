#include "suite_rows.h"
#include "robot_cases.h"
#include "inverse_kinematics_cases.h"

#include "inverse_kinematics/capabilities.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/manipulator/robot.h>
#include <praxis/manipulator/kinematics.h>
#include <praxis/manipulator/screw_chain.h>

#include <praxis/manipulator/baseline/kinematics.h>

#include <praxis/rigid_motion/screw.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <cstddef>
#include <string_view>

namespace ais4104 {
namespace {

answer<void> book_tolerance_solve(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                  const praxis::manipulator::forward_kinematics_ops &forward, const praxis::manipulator::differential_kinematics_ops &differential,
                                  const praxis::optimization::solver_ops &solver, const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired,
                                  const Eigen::VectorXd &j0, const praxis::manipulator::solver_parameters &, praxis::manipulator::ik_result &result)
{
    const praxis::manipulator::solver_parameters book(book_position_tolerance, book_orientation_tolerance, robot_iteration_cap);

    return reference_manipulator().ik.inverse_kinematics(screw, frames, forward, differential, solver, chain, desired, j0, book, result);
}

answer<praxis::manipulator::kinematics> solver_of(std::size_t subject)
{
    const praxis::manipulator::capabilities &reference = reference_manipulator();

    return praxis::manipulator::make_kinematics(chain_of(frozen_chains[subject]), reference.fk, reference.dk,
                                                praxis::manipulator::inverse_kinematics_ops{.inverse_kinematics = &book_tolerance_solve}, reference_motions().screw,
                                                reference_motions().frame, reference_solving().solver);
}

std::string composition_line(std::string_view subject, const answer<praxis::manipulator::kinematics> &solver)
{
    if(solver)
        return std::format("make_kinematics composed the {} chain", subject);

    return std::format("make_kinematics refused the {} chain ({})", subject, refusal_name(solver.error()));
}

const praxis::manipulator::robot_ops &student_frames()
{
    static const praxis::manipulator::robot_ops frames = manipulator::end_effector_frames();

    return frames;
}

answer<Eigen::VectorXd> ik_solve_flange_pose_of(const praxis::manipulator::kinematics &solver, const Eigen::Matrix4d &target, const Eigen::VectorXd &seed)
{
    return student_frames().ik_solve_flange_pose(solver, target, seed);
}

answer<Eigen::VectorXd> ik_solve_pose_of(const praxis::manipulator::kinematics &solver, const Eigen::Matrix4d &target, const Eigen::VectorXd &seed, const Eigen::Matrix4d &offset)
{
    return student_frames().ik_solve_pose(solver, reference_motions().frame, target, seed, offset);
}

case_reading pose_reached(const Eigen::VectorXd &answered, const robot_solve_case &each, const pinned_bound &bound)
{
    const praxis::manipulator::screw_chain chain = chain_of(frozen_chains[each.subject]);
    const answer<Eigen::Matrix4d> flange         = reference_manipulator().fk.forward_kinematics(reference_motions().screw, chain.home, chain.space_screws, answered);
    if(!flange)
        return case_reading{false, std::format(" answered a configuration the reference forward kinematics refused ({})", refusal_name(flange.error()))};

    const answer<Eigen::Matrix4d> reached = each.slot == 0 ? flange : reference_manipulator().robot.tool_pose_from_flange_pose(*flange, matrix_of(each.offset));
    if(!reached)
        return case_reading{false, std::format(" answered a configuration whose tool pose the reference refused ({})", refusal_name(reached.error()))};

    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const praxis::evaluation::residual seen          = praxis::evaluation::pose_residual(*reached, matrix_of(each.target));
    if(seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres)
        return case_reading{true, residual_clause(seen, allowed)};

    return case_reading{false, " converged off target" + residual_clause(seen, allowed)};
}

case_reading reach_of(const answer<Eigen::VectorXd> &solved, const robot_solve_case &each, const pinned_bound &bound)
{
    if(!solved && solved.error() == praxis::refusal::no_solution)
        return case_reading{false, " did not converge (no_solution)"};
    if(!solved)
        return case_reading{false, refused_clause(solved.error())};

    const std::size_t joints = frozen_chains[each.subject].joints;
    if(static_cast<std::size_t>(solved->size()) != joints)
        return case_reading{false, std::format(" answered {} joints for a chain of {}", solved->size(), joints)};
    if(!solved->allFinite())
        return case_reading{false, " non-finite"};

    return pose_reached(*solved, each, bound);
}

answer<Eigen::VectorXd> solved_by(const praxis::manipulator::kinematics &solver, const robot_solve_case &each)
{
    const Eigen::Matrix4d target = matrix_of(each.target);
    const Eigen::VectorXd seed   = leading_entries(each.seed, frozen_chains[each.subject].joints);
    if(each.slot == 0)
        return ik_solve_flange_pose_of(solver, target, seed);

    return ik_solve_pose_of(solver, target, seed, matrix_of(each.offset));
}

void check_case(const graded_row &row, const robot_solve_case &each, std::size_t index)
{
    const std::string_view subject                       = subject_names[each.subject];
    const answer<praxis::manipulator::kinematics> solver = solver_of(each.subject);
    {
        INFO(composition_line(subject, solver));
        REQUIRE(solver.has_value());
    }

    const case_reading read = reach_of(solved_by(*solver, each), each, row.bound);
    INFO(row_line(row.task, row.slot, subject, index, read.clause));
    CHECK(read.passed);
}

std::size_t run_slot(const graded_row &row, std::size_t slot)
{
    std::array<std::size_t, subject_names.size()> numbered{};
    std::size_t count = 0;
    for(const robot_solve_case &each : robot_solve_cases)
    {
        if(each.slot != slot)
            continue;
        check_case(row, each, numbered[each.subject]++);
        ++count;
    }
    return count;
}

std::size_t run_ik_solve_flange_pose(const graded_row &row)
{
    return run_slot(row, 0);
}

std::size_t run_ik_solve_pose(const graded_row &row)
{
    return run_slot(row, 1);
}

constexpr std::array<graded_row, 2> robot_solve_rows{{
        graded_row{"T.2d", "end_effector_frames.ik_solve_flange_pose", "robot.ik_solve_flange_pose", ik_solve_flange_pose_bound, &run_ik_solve_flange_pose},
        graded_row{"T.2e", "end_effector_frames.ik_solve_pose", "robot.ik_solve_pose", ik_solve_pose_bound, &run_ik_solve_pose},
}};

}
}

TEST_CASE("every graded robot solve reaches its frozen pose", "[robot][graded]")
{
    for(const ais4104::graded_row &row : ais4104::robot_solve_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row);
        }
    }
}

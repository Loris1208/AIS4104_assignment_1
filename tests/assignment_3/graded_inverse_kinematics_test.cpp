#include "suite_rows.h"
#include "iteration_record.h"
#include "inverse_kinematics_cases.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/manipulator/kinematics.h>
#include <praxis/manipulator/screw_chain.h>
#include <praxis/manipulator/solver_adapters.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <cstddef>

namespace ais4104 {
namespace {

using inverse_call = answer<void> (*)(const praxis::manipulator::screw_chain &, const Eigen::Matrix4d &, const Eigen::VectorXd &, praxis::manipulator::ik_result &);

answer<void> body_route_of(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed, praxis::manipulator::ik_result &result)
{
    const praxis::manipulator::capabilities &reference = reference_manipulator();

    return praxis::manipulator::body_inverse_kinematics(reference_motions().screw, reference_motions().frame, reference.fk, reference.dk, student_solving().solver, chain, desired, seed,
                                                        solver_route_parameters(), result);
}

answer<void> space_route_of(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed, praxis::manipulator::ik_result &result)
{
    const praxis::manipulator::capabilities &reference = reference_manipulator();

    return praxis::manipulator::space_inverse_kinematics(reference_motions().screw, reference_motions().frame, reference.fk, reference.dk, student_solving().solver, chain, desired,
                                                         seed, solver_route_parameters(), result);
}

case_reading pose_reached(const Eigen::VectorXd &answered, const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const pinned_bound &bound)
{
    const answer<Eigen::Matrix4d> reached = reference_manipulator().fk.forward_kinematics(reference_motions().screw, chain.home, chain.space_screws, answered);
    if(!reached)
        return case_reading{false, std::format(" answered a configuration the reference forward kinematics refused ({})", refusal_name(reached.error()))};

    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const praxis::evaluation::residual seen          = praxis::evaluation::pose_residual(*reached, desired);
    if(seen.magnitude <= allowed.magnitude && seen.linear_error_metres <= allowed.linear_metres)
        return case_reading{true, residual_clause(seen, allowed)};

    return case_reading{false, " converged off target" + residual_clause(seen, allowed)};
}

case_reading reach_of(const answer<void> &solved, const praxis::manipulator::ik_result &result, const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired,
                      const pinned_bound &bound)
{
    if(!solved && solved.error() == praxis::refusal::no_solution)
        return case_reading{false, " did not converge (no_solution)"};
    if(!solved)
        return case_reading{false, refused_clause(solved.error())};
    if(result.solutions.empty())
        return case_reading{false, " did not converge (answered no configuration)"};

    const Eigen::VectorXd &answered = result.solutions.front();
    if(static_cast<std::size_t>(answered.size()) != chain.joint_count())
        return case_reading{false, std::format(" answered {} joints for a chain of {}", answered.size(), chain.joint_count())};
    if(!answered.allFinite())
        return case_reading{false, " non-finite"};

    return pose_reached(answered, chain, desired, bound);
}

void check_case(const graded_row &row, inverse_call solving, const ik_case &each, std::size_t index)
{
    const praxis::manipulator::screw_chain chain = chain_of(frozen_chains[each.subject]);
    const Eigen::Matrix4d desired                = target_of(each);
    const Eigen::VectorXd seed                   = seed_of(each);
    praxis::manipulator::ik_result result;
    const answer<void> solved     = solving(chain, desired, seed, result);
    const case_reading read       = reach_of(solved, result, chain, desired, row.bound);
    const record_reading recorded = record_of(chain, desired, seed, solved, result);

    {
        INFO(row_line(row.task, row.slot, subject_names[each.subject], index, read.clause));
        CHECK(read.passed);
    }
    {
        INFO(row_line(row.task, row.slot, subject_names[each.subject], index, recorded.clause));
        CHECK(recorded.consistent);
    }
}

std::size_t run_cases(const graded_row &row, inverse_call solving)
{
    std::array<std::size_t, subject_names.size()> numbered{};
    for(const ik_case &each : ik_cases)
        check_case(row, solving, each, numbered[each.subject]++);

    return ik_cases.size();
}

std::size_t run_body_route(const graded_row &row)
{
    return run_cases(row, &body_route_of);
}

std::size_t run_space_route(const graded_row &row)
{
    return run_cases(row, &space_route_of);
}

constexpr std::array<graded_row, 2> solver_rows{{
        graded_row{"T.1a", "solvers.newton_raphson body", "solver.solve", reach_bound, &run_body_route},
        graded_row{"T.1a", "solvers.newton_raphson space", "solver.solve", reach_bound, &run_space_route},
}};

}
}

TEST_CASE("every graded solver drives the arm to its frozen target", "[solver][graded]")
{
    for(const ais4104::graded_row &row : ais4104::solver_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row, "manipulator");
        }
    }
}

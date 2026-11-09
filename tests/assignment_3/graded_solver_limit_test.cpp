#include "suite_rows.h"
#include "iteration_record.h"
#include "inverse_kinematics_cases.h"

#include <praxis/manipulator/kinematics.h>
#include <praxis/manipulator/screw_chain.h>
#include <praxis/manipulator/solver_adapters.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <cstddef>
#include <string_view>

namespace ais4104 {
namespace {

using adapter = decltype(praxis::manipulator::inverse_kinematics_ops::inverse_kinematics);

struct limit_row
{
    std::string_view task;
    std::string_view slot;
    adapter route;
};

constexpr std::array<limit_row, 2> limit_rows{{
        limit_row{"T.1a", "solvers.newton_raphson body at its limit", &praxis::manipulator::body_inverse_kinematics},
        limit_row{"T.1a", "solvers.newton_raphson space at its limit", &praxis::manipulator::space_inverse_kinematics},
}};

praxis::manipulator::solver_parameters limit_parameters()
{
    return praxis::manipulator::solver_parameters(book_position_tolerance, book_orientation_tolerance, solver_limit_cap);
}

std::size_t index_within_subject(std::size_t index)
{
    std::size_t within = 0;
    for(std::size_t k = 0; k < index; ++k)
        within += ik_cases[k].subject == ik_cases[index].subject ? 1 : 0;

    return within;
}

case_reading stopped_at_limit(const answer<void> &solved, const record_reading &kept)
{
    if(solved)
        return case_reading{false, std::format(" answered within a limit of {} steps", solver_limit_cap)};
    if(solved.error() != praxis::refusal::no_solution)
        return case_reading{false, refused_clause(solved.error())};

    return case_reading{kept.consistent, kept.clause};
}

void check_limit_case(const limit_row &row, std::size_t index)
{
    const ik_case &each                                = ik_cases[index];
    const praxis::manipulator::screw_chain chain       = chain_of(frozen_chains[each.subject]);
    const Eigen::Matrix4d desired                      = target_of(each);
    const Eigen::VectorXd seed                         = seed_of(each);
    const praxis::manipulator::capabilities &reference = reference_manipulator();
    praxis::manipulator::ik_result result;

    const answer<void> solved =
            row.route(reference_motions().screw, reference_motions().frame, reference.fk, reference.dk, student_solving().solver, chain, desired, seed, limit_parameters(), result);
    const case_reading read = stopped_at_limit(solved, kept_record_of(chain, desired, seed, result));

    INFO(row_line(row.task, row.slot, subject_names[each.subject], index_within_subject(index), read.clause));
    CHECK(read.passed);
}

}
}

TEST_CASE("every solver stopped at its limit keeps its iterates", "[solver][graded]")
{
    for(const ais4104::limit_row &row : ais4104::limit_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            for(const std::size_t index : ais4104::solver_limit_cases)
                ais4104::check_limit_case(row, index);
        }
    }
}

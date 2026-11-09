#include "suite_rows.h"
#include "root_finding_cases.h"

#include <praxis/optimization/problem.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/QR>
#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <algorithm>
#include <initializer_list>

namespace ais4104 {
namespace {

struct root_run
{
    answer<Eigen::VectorXd> answered;
    std::vector<Eigen::VectorXd> iterates;
};

root_run solve(const praxis::optimization::problem &p, const Eigen::VectorXd &seed, std::uint32_t cap)
{
    root_run run{praxis::unexpected(praxis::refusal::not_implemented), {}};
    run.answered = student_solving().solver.solve(p, seed, cap, run.iterates);

    return run;
}

std::optional<Eigen::VectorXd> pseudoinverse_step(const praxis::optimization::problem &p, const Eigen::VectorXd &x)
{
    const answer<Eigen::VectorXd> e = p.error(x);
    if(!e)
        return std::nullopt;
    const answer<Eigen::MatrixXd> j = p.jacobian(x);
    if(!j)
        return std::nullopt;

    return Eigen::VectorXd(Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*j).solve(*e));
}

// Every iterate after the seed is x + J^+ e of the one before it, and none before the last is converged.
case_reading steps_of(const praxis::optimization::problem &p, const Eigen::VectorXd &seed, const std::vector<Eigen::VectorXd> &iterates)
{
    if(iterates.empty() || iterates.front().size() != seed.size() || iterates.front() != seed)
        return case_reading{false, " kept iterates that do not begin with the seed"};

    for(std::size_t k = 1; k < iterates.size(); ++k)
    {
        const Eigen::VectorXd &before   = iterates[k - 1];
        const answer<Eigen::VectorXd> e = p.error(before);
        if(e && p.converged(*e))
            return case_reading{false, std::format(" stepped on from iterate {}, which is already converged", k - 1)};
        const std::optional<Eigen::VectorXd> step = pseudoinverse_step(p, before);
        if(!step || iterates[k].size() != step->size())
            return case_reading{false, std::format(" kept an iterate {} of another size than the one before it", k)};
        const double difference = (iterates[k] - (before + *step)).lpNorm<Eigen::Infinity>();
        const double allowed    = root_step_absolute + root_step_relative * std::max(1.0, step->lpNorm<Eigen::Infinity>());
        if(!(difference <= allowed))
            return case_reading{false, std::format(" kept an iterate {} that is not x + J^+ e of the one before it", k)};
    }

    return case_reading{true, ""};
}

case_reading root_reading(const praxis::optimization::problem &p, const Eigen::VectorXd &seed, const root_run &run, const pinned_bound &bound)
{
    if(!run.answered)
        return case_reading{false, refused_clause(run.answered.error())};
    if(run.answered->size() != seed.size() || !run.answered->allFinite())
        return case_reading{false, std::format(" answered {} coordinates for a problem of {}, or a non-finite one", run.answered->size(), seed.size())};
    if(run.iterates.empty() || run.iterates.back() != *run.answered)
        return case_reading{false, " answered another point than the last iterate it kept"};

    const answer<Eigen::VectorXd> e = p.error(*run.answered);
    if(!e)
        return case_reading{false, " answered a point where its problem refuses the error"};
    const double error = e->lpNorm<Eigen::Infinity>();
    if(!(error <= bound.magnitude))
        return case_reading{false, std::format(" answered a point whose error is {:.6e}, allowed {:.6e}", error, bound.magnitude)};

    return steps_of(p, seed, run.iterates);
}

std::size_t run_newton_raphson(const graded_row &row)
{
    for(std::size_t index = 0; index < root_finding_cases.size(); ++index)
    {
        const root_finding_case &each         = root_finding_cases[index];
        const root_finding_problem &problem   = root_finding_problems[each.problem];
        const praxis::optimization::problem p = problem_of(problem);
        const root_run run                    = solve(p, seed_of(problem, each), root_iteration_cap);
        const case_reading read               = root_reading(p, seed_of(problem, each), run, row.bound);

        INFO(row_line(row.task, row.slot, problem.name, index, read.clause));
        CHECK(read.passed);
    }

    return root_finding_cases.size();
}

case_reading stopped_at_limit(const praxis::optimization::problem &p, const Eigen::VectorXd &seed, std::uint32_t cap)
{
    const root_run run = solve(p, seed, cap);
    if(run.answered)
        return case_reading{false, std::format(" answered within a limit of {} steps", cap)};
    if(run.answered.error() != praxis::refusal::no_solution)
        return case_reading{false, refused_clause(run.answered.error())};
    if(run.iterates.size() != cap + 1u)
        return case_reading{false, std::format(" kept {} iterates for {} steps", run.iterates.size(), cap)};

    return steps_of(p, seed, run.iterates);
}

// The refused problem is solved; its iterates are read against the unrefused one.
case_reading refusal_passed_on(const praxis::optimization::problem &refused, std::size_t kept)
{
    const root_finding_problem &problem = root_finding_problems[root_finding_cases[0].problem];
    const Eigen::VectorXd seed          = seed_of(problem, root_finding_cases[0]);
    const root_run run                  = solve(refused, seed, root_iteration_cap);
    if(run.answered)
        return case_reading{false, " answered a point"};
    if(run.answered.error() != praxis::refusal::degenerate)
        return case_reading{false, refused_clause(run.answered.error())};
    if(run.iterates.size() != kept)
        return case_reading{false, std::format(" kept {} iterates, not {}", run.iterates.size(), kept)};

    return steps_of(problem_of(problem), seed, run.iterates);
}

constexpr std::array<graded_row, 1> newton_raphson_rows{{
        graded_row{"T.1a", "solvers.newton_raphson", "solver.solve", converged_bound, &run_newton_raphson},
}};

}
}

TEST_CASE("every graded Newton-Raphson solve answers a root its problem accepts", "[solver][graded]")
{
    for(const ais4104::graded_row &row : ais4104::newton_raphson_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row, "optimization");
        }
    }
}

TEST_CASE("every Newton-Raphson solve stopped at its limit keeps its iterates", "[solver][graded]")
{
    SECTION("task T.1a - solvers.newton_raphson at its limit")
    {
        for(const std::size_t index : ais4104::root_limit_cases)
        {
            const ais4104::root_finding_case &each       = ais4104::root_finding_cases[index];
            const ais4104::root_finding_problem &problem = ais4104::root_finding_problems[each.problem];
            const ais4104::case_reading read             = ais4104::stopped_at_limit(ais4104::problem_of(problem), ais4104::seed_of(problem, each), ais4104::root_limit_cap);

            INFO(ais4104::row_line("T.1a", "solvers.newton_raphson at its limit", problem.name, index, read.clause));
            CHECK(read.passed);
        }
    }
    SECTION("task T.1a - solvers.newton_raphson with no root to reach")
    {
        const Eigen::VectorXd seed = Eigen::VectorXd::Constant(1, ais4104::no_root_seed);
        for(const std::uint32_t cap : {ais4104::root_limit_cap, ais4104::root_iteration_cap})
        {
            const ais4104::case_reading read = ais4104::stopped_at_limit(ais4104::problem_of(ais4104::no_root_problem), seed, cap);

            INFO(std::format("task T.1a - solvers.newton_raphson {} stopped at {} steps{}", ais4104::no_root_problem.name, cap, read.clause));
            CHECK(read.passed);
        }
    }
}

TEST_CASE("a Newton-Raphson solve answers the refusal its problem gave", "[solver][graded]")
{
    SECTION("task T.1a - solvers.newton_raphson with a refused error")
    {
        const ais4104::case_reading read = ais4104::refusal_passed_on(ais4104::error_refused_problem(), 2);

        INFO("task T.1a - solvers.newton_raphson with a refused error" << read.clause);
        CHECK(read.passed);
    }
    SECTION("task T.1a - solvers.newton_raphson with a refused Jacobian")
    {
        const ais4104::case_reading read = ais4104::refusal_passed_on(ais4104::jacobian_refused_problem(), 1);

        INFO("task T.1a - solvers.newton_raphson with a refused Jacobian" << read.clause);
        CHECK(read.passed);
    }
}

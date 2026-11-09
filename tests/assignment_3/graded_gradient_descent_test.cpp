#include "suite_rows.h"
#include "minimizer_cases.h"

#include <praxis/optimization/problem.h>
#include <praxis/optimization/minimizer.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>

namespace ais4104 {
namespace {

struct descent_run
{
    answer<Eigen::VectorXd> answered;
    std::vector<Eigen::VectorXd> iterates;
};

praxis::optimization::problem problem_of(const descent_objective &objective)
{
    return praxis::optimization::problem{
            .gradient   = [gradient = objective.gradient](const Eigen::VectorXd &x) -> answer<Eigen::VectorXd> { return gradient(x); },
            .stationary = [](const Eigen::VectorXd &g) { return g.lpNorm<Eigen::Infinity>() < stationary_magnitude; },
    };
}

Eigen::VectorXd seed_of(const descent_case &each)
{
    return Eigen::Map<const Eigen::VectorXd>(each.seed.data(), static_cast<Eigen::Index>(each.dimension));
}

descent_run descend(const praxis::optimization::problem &p, const Eigen::VectorXd &seed, double step_length, std::uint32_t cap)
{
    descent_run run{praxis::unexpected(praxis::refusal::not_implemented), {}};
    run.answered = student_solving().minimizer.gradient_descent(p, seed, step_length, cap, run.iterates);

    return run;
}

// Every iterate after the seed is the fixed-length step from the one before it, and none before the last is stationary.
case_reading steps_of(const descent_objective &objective, const Eigen::VectorXd &seed, double step_length, const std::vector<Eigen::VectorXd> &iterates)
{
    if(iterates.empty() || iterates.front().size() != seed.size() || iterates.front() != seed)
        return case_reading{false, " kept iterates that do not begin with the seed"};

    for(std::size_t k = 1; k < iterates.size(); ++k)
    {
        const Eigen::VectorXd &before = iterates[k - 1];
        if(objective.gradient(before).lpNorm<Eigen::Infinity>() < stationary_magnitude)
            return case_reading{false, std::format(" stepped on from iterate {}, where the gradient is already stationary", k - 1)};
        const Eigen::VectorXd expected = before - step_length * objective.gradient(before);
        if(iterates[k].size() != expected.size() || (iterates[k] - expected).lpNorm<Eigen::Infinity>() > descent_step_absolute)
            return case_reading{false, std::format(" kept an iterate {} that is not x - gamma grad f(x) of the one before it", k)};
    }

    return case_reading{true, ""};
}

case_reading stationary_reading(const descent_objective &objective, const descent_case &each, const descent_run &run, const pinned_bound &bound)
{
    if(!run.answered)
        return case_reading{false, refused_clause(run.answered.error())};
    if(run.answered->size() != seed_of(each).size() || !run.answered->allFinite())
        return case_reading{false, std::format(" answered {} coordinates for a function of {}, or a non-finite one", run.answered->size(), each.dimension)};
    if(run.iterates.empty() || run.iterates.back() != *run.answered)
        return case_reading{false, " answered another point than the last iterate it kept"};

    const double gradient = objective.gradient(*run.answered).lpNorm<Eigen::Infinity>();
    if(gradient > bound.magnitude)
        return case_reading{false, std::format(" answered a point whose gradient is {:.6e}, allowed {:.6e}", gradient, bound.magnitude)};

    return steps_of(objective, seed_of(each), each.step_length, run.iterates);
}

std::size_t run_gradient_descent(const graded_row &row)
{
    for(std::size_t index = 0; index < descent_cases.size(); ++index)
    {
        const descent_case &each           = descent_cases[index];
        const descent_objective &objective = descent_objectives[each.objective];
        const descent_run run              = descend(problem_of(objective), seed_of(each), each.step_length, descent_iteration_cap);
        const case_reading read            = stationary_reading(objective, each, run, row.bound);

        INFO(row_line(row.task, row.slot, objective.name, index, read.clause));
        CHECK(read.passed);
    }

    return descent_cases.size();
}

case_reading stopped_at_limit(const descent_objective &objective, const descent_case &each, const descent_run &run)
{
    if(run.answered)
        return case_reading{false, std::format(" answered within a limit of {} steps", descent_limit_cap)};
    if(run.answered.error() != praxis::refusal::no_solution)
        return case_reading{false, refused_clause(run.answered.error())};
    if(run.iterates.size() != descent_limit_cap + 1u)
        return case_reading{false, std::format(" kept {} iterates for {} steps", run.iterates.size(), descent_limit_cap)};

    return steps_of(objective, seed_of(each), each.step_length, run.iterates);
}

// The gradient refuses where x < 0: from 4, a step of 0.75 lands on -0.5 and the descent must answer that refusal.
answer<Eigen::VectorXd> gradient_refused_below_zero(const Eigen::VectorXd &x)
{
    if(x[0] < 0.0)
        return praxis::unexpected(praxis::refusal::degenerate);

    return shifted_parabola_gradient(x);
}

constexpr std::array<graded_row, 1> gradient_descent_rows{{
        graded_row{"T.1b", "solvers.gradient_descent", "minimizer.gradient_descent", stationary_bound, &run_gradient_descent},
}};

}
}

TEST_CASE("every graded gradient descent steps to a stationary point", "[minimizer][graded]")
{
    for(const ais4104::graded_row &row : ais4104::gradient_descent_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row, "optimization");
        }
    }
}

TEST_CASE("every gradient descent stopped at its limit keeps its iterates", "[minimizer][graded]")
{
    SECTION("task T.1b - solvers.gradient_descent at its limit")
    {
        for(const std::size_t index : ais4104::descent_limit_cases)
        {
            const ais4104::descent_case &each           = ais4104::descent_cases[index];
            const ais4104::descent_objective &objective = ais4104::descent_objectives[each.objective];
            const ais4104::descent_run run              = ais4104::descend(ais4104::problem_of(objective), ais4104::seed_of(each), each.step_length, ais4104::descent_limit_cap);
            const ais4104::case_reading read            = ais4104::stopped_at_limit(objective, each, run);

            INFO(ais4104::row_line("T.1b", "solvers.gradient_descent at its limit", objective.name, index, read.clause));
            CHECK(read.passed);
        }
    }
}

TEST_CASE("a gradient descent answers the refusal its gradient gave", "[minimizer][graded]")
{
    SECTION("task T.1b - solvers.gradient_descent with a refused gradient")
    {
        const ais4104::descent_objective objective{"shifted parabola", &ais4104::shifted_parabola_gradient};
        const praxis::optimization::problem p{
                .gradient   = &ais4104::gradient_refused_below_zero,
                .stationary = [](const Eigen::VectorXd &g) { return g.lpNorm<Eigen::Infinity>() < ais4104::stationary_magnitude; },
        };
        const Eigen::VectorXd seed     = Eigen::VectorXd::Constant(1, 4.0);
        const ais4104::descent_run run = ais4104::descend(p, seed, 0.75, ais4104::descent_iteration_cap);

        INFO("task T.1b - solvers.gradient_descent with a refused gradient answered "
             << (run.answered ? std::string("a point") : std::string(ais4104::refusal_name(run.answered.error()))) << " with " << run.iterates.size() << " iterates kept");
        CHECK((!run.answered && run.answered.error() == praxis::refusal::degenerate));
        CHECK(run.iterates.size() == 2);
        CHECK(ais4104::steps_of(objective, seed, 0.75, run.iterates).passed);
    }
}

#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_3_ROOT_FINDING_CASES_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_3_ROOT_FINDING_CASES_H

#include "suite_rows.h"

#include <praxis/optimization/problem.h>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ais4104 {

// Each problem poses e(x) = x_d - f(x) with x_d = 0 and J = df/dx, so a root of f is a point where e vanishes.

// e(x) = x^2 - 1
inline Eigen::VectorXd two_roots_error(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, -(1.0 - x[0] * x[0]));
}

inline Eigen::MatrixXd two_roots_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::MatrixXd::Constant(1, 1, -2.0 * x[0]);
}

// e(x) = 5 + 2x - x^3
inline Eigen::VectorXd cubic_error(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, -(x[0] * x[0] * x[0] - 2.0 * x[0] - 5.0));
}

inline Eigen::MatrixXd cubic_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::MatrixXd::Constant(1, 1, 3.0 * x[0] * x[0] - 2.0);
}

// e(x, y) = (4 - x^2 - y^2, 1 - xy)
inline Eigen::VectorXd square_system_error(const Eigen::VectorXd &x)
{
    return -Eigen::Vector2d(x[0] * x[0] + x[1] * x[1] - 4.0, x[0] * x[1] - 1.0);
}

inline Eigen::MatrixXd square_system_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::Matrix2d{{2.0 * x[0], 2.0 * x[1]}, {x[1], x[0]}};
}

// e(x, y) = x^2 + y^2 - 1
inline Eigen::VectorXd unit_circle_error(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, -(1.0 - x[0] * x[0] - x[1] * x[1]));
}

inline Eigen::MatrixXd unit_circle_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::RowVector2d(-2.0 * x[0], -2.0 * x[1]);
}

// e(x, y) = 0.75 - cos(x) - 0.5 cos(x + y)
inline Eigen::VectorXd two_link_error(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, -(std::cos(x[0]) + 0.5 * std::cos(x[0] + x[1]) - 0.75));
}

inline Eigen::MatrixXd two_link_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::RowVector2d(-std::sin(x[0]) - 0.5 * std::sin(x[0] + x[1]), -0.5 * std::sin(x[0] + x[1]));
}

// e(x) = (1 - x, 1 - x^2)
inline Eigen::VectorXd tall_line_error(const Eigen::VectorXd &x)
{
    return -Eigen::Vector2d(x[0] - 1.0, x[0] * x[0] - 1.0);
}

inline Eigen::MatrixXd tall_line_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::Vector2d(1.0, 2.0 * x[0]);
}

// e(x) = (4 - x^2, 8 - x^3)
inline Eigen::VectorXd tall_cubic_error(const Eigen::VectorXd &x)
{
    return -Eigen::Vector2d(x[0] * x[0] - 4.0, x[0] * x[0] * x[0] - 8.0);
}

inline Eigen::MatrixXd tall_cubic_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::Vector2d(2.0 * x[0], 3.0 * x[0] * x[0]);
}

// e(x) = 2x - 2 - x^3
inline Eigen::VectorXd cycle_error(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, -(x[0] * x[0] * x[0] - 2.0 * x[0] + 2.0));
}

inline Eigen::MatrixXd cycle_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::MatrixXd::Constant(1, 1, 3.0 * x[0] * x[0] - 2.0);
}

struct root_finding_problem
{
    std::string_view name;
    Eigen::Index unknowns;
    Eigen::VectorXd (*error)(const Eigen::VectorXd &x);
    Eigen::MatrixXd (*jacobian)(const Eigen::VectorXd &x);
};

constexpr std::array<root_finding_problem, 7> root_finding_problems{{
        root_finding_problem{"two roots", 1, &two_roots_error, &two_roots_jacobian},
        root_finding_problem{"cubic", 1, &cubic_error, &cubic_jacobian},
        root_finding_problem{"square system", 2, &square_system_error, &square_system_jacobian},
        root_finding_problem{"unit circle", 2, &unit_circle_error, &unit_circle_jacobian},
        root_finding_problem{"two link", 2, &two_link_error, &two_link_jacobian},
        root_finding_problem{"tall line", 1, &tall_line_error, &tall_line_jacobian},
        root_finding_problem{"tall cubic", 1, &tall_cubic_error, &tall_cubic_jacobian},
}};

// Newton-Raphson from its seed reaches no root of this problem; it cycles.
constexpr root_finding_problem no_root_problem{"cycle", 1, &cycle_error, &cycle_jacobian};

// Only the first `unknowns` coordinates of the seed are read.
struct root_finding_case
{
    std::size_t problem;
    std::array<double, 2> seed;
};

constexpr std::array<root_finding_case, 27> root_finding_cases{
        root_finding_case{.problem = 0, .seed = {0.5, 0.0}},   root_finding_case{.problem = 0, .seed = {-0.5, 0.0}}, root_finding_case{.problem = 0, .seed = {3.0, 0.0}},
        root_finding_case{.problem = 0, .seed = {-2.0, 0.0}},  root_finding_case{.problem = 1, .seed = {3.0, 0.0}},  root_finding_case{.problem = 1, .seed = {-3.0, 0.0}},
        root_finding_case{.problem = 1, .seed = {1.5, 0.0}},   root_finding_case{.problem = 1, .seed = {4.0, 0.0}},  root_finding_case{.problem = 2, .seed = {2.0, 0.5}},
        root_finding_case{.problem = 2, .seed = {-1.8, -0.7}}, root_finding_case{.problem = 2, .seed = {0.5, 2.5}},  root_finding_case{.problem = 2, .seed = {3.0, 1.0}},
        root_finding_case{.problem = 3, .seed = {2.0, 1.0}},   root_finding_case{.problem = 3, .seed = {-3.0, 2.5}}, root_finding_case{.problem = 3, .seed = {0.5, 0.25}},
        root_finding_case{.problem = 3, .seed = {-1.0, -2.0}}, root_finding_case{.problem = 4, .seed = {0.5, 0.5}},  root_finding_case{.problem = 4, .seed = {1.0, -1.0}},
        root_finding_case{.problem = 4, .seed = {2.0, 0.5}},   root_finding_case{.problem = 5, .seed = {3.0, 0.0}},  root_finding_case{.problem = 5, .seed = {-2.0, 0.0}},
        root_finding_case{.problem = 5, .seed = {0.5, 0.0}},   root_finding_case{.problem = 5, .seed = {2.0, 0.0}},  root_finding_case{.problem = 6, .seed = {3.0, 0.0}},
        root_finding_case{.problem = 6, .seed = {1.0, 0.0}},   root_finding_case{.problem = 6, .seed = {1.5, 0.0}},  root_finding_case{.problem = 6, .seed = {4.0, 0.0}},
};

// The cases from which the solve is stopped at root_limit_cap steps.
constexpr std::array<std::size_t, 27> root_limit_cases = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26};

constexpr std::uint32_t root_iteration_cap = 20;
constexpr std::uint32_t root_limit_cap     = 2;

constexpr double no_root_seed = 0.0;

// The greatest entry of e at the answer, twice; the convergence test of every problem.
constexpr double converged_magnitude = 1e-12;
constexpr double converged_linear    = 1e-12;

constexpr pinned_bound converged_bound{converged_magnitude, converged_linear, bound_standing::at_the_platform, bound_standing::at_the_platform};

// An iterate may differ from x + J^+ e of the one before it by root_step_absolute plus root_step_relative
// times the larger of one and the step's greatest entry.
constexpr double root_step_absolute = 1e-7;
constexpr double root_step_relative = 1e-7;

// The refused-error problem refuses every x above this; the seed of case 0 lies below it.
constexpr double refused_error_boundary = 0.9;

inline Eigen::VectorXd seed_of(const root_finding_problem &problem, const root_finding_case &each)
{
    return Eigen::Map<const Eigen::VectorXd>(each.seed.data(), problem.unknowns);
}

inline praxis::optimization::problem problem_of(const root_finding_problem &each)
{
    return praxis::optimization::problem{
            .error     = [error = each.error](const Eigen::VectorXd &x) -> answer<Eigen::VectorXd> { return error(x); },
            .jacobian  = [jacobian = each.jacobian](const Eigen::VectorXd &x) -> answer<Eigen::MatrixXd> { return jacobian(x); },
            .converged = [](const Eigen::VectorXd &e) { return e.lpNorm<Eigen::Infinity>() <= converged_magnitude; },
    };
}

inline praxis::optimization::problem error_refused_problem()
{
    const root_finding_problem &each = root_finding_problems[root_finding_cases[0].problem];
    praxis::optimization::problem p  = problem_of(each);
    p.error                          = [error = each.error](const Eigen::VectorXd &x) -> answer<Eigen::VectorXd>
    {
        if(x[0] > refused_error_boundary)
            return praxis::unexpected(praxis::refusal::degenerate);

        return error(x);
    };

    return p;
}

inline praxis::optimization::problem jacobian_refused_problem()
{
    praxis::optimization::problem p = problem_of(root_finding_problems[root_finding_cases[0].problem]);
    p.jacobian                      = [](const Eigen::VectorXd &) -> answer<Eigen::MatrixXd> { return praxis::unexpected(praxis::refusal::degenerate); };

    return p;
}

}

#endif

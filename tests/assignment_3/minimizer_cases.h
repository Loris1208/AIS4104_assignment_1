#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_3_MINIMIZER_CASES_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_3_MINIMIZER_CASES_H

#include "suite_rows.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ais4104 {

// f(x) = (x - 1)^2
inline Eigen::VectorXd shifted_parabola_gradient(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, 2.0 * (x[0] - 1.0));
}

// f(x, y) = (x^2 + 4 y^2) / 2
inline Eigen::VectorXd oval_bowl_gradient(const Eigen::VectorXd &x)
{
    return Eigen::Vector2d(x[0], 4.0 * x[1]);
}

// f(x, y) = x^4 / 4 - x^2 / 2 + y^2 / 2
inline Eigen::VectorXd double_well_gradient(const Eigen::VectorXd &x)
{
    return Eigen::Vector2d(x[0] * x[0] * x[0] - x[0], x[1]);
}

struct descent_objective
{
    std::string_view name;
    Eigen::VectorXd (*gradient)(const Eigen::VectorXd &x);
};

constexpr std::array<descent_objective, 3> descent_objectives{{
        descent_objective{"shifted parabola", &shifted_parabola_gradient},
        descent_objective{"oval bowl", &oval_bowl_gradient},
        descent_objective{"double well", &double_well_gradient},
}};

// Only the first `dimension` coordinates of the seed are read.
struct descent_case
{
    std::size_t objective;
    std::array<double, 2> seed;
    std::size_t dimension;
    double step_length;
};

constexpr std::array<descent_case, 12> descent_cases{{
        descent_case{0, {3.0, 0.0}, 1, 0.1},     descent_case{0, {-2.5, 0.0}, 1, 0.25}, descent_case{0, {4.0, 0.0}, 1, 0.5},
        descent_case{0, {-0.75, 0.0}, 1, 0.9},   descent_case{1, {3.0, -2.0}, 2, 0.1},  descent_case{1, {-1.5, 2.5}, 2, 0.2},
        descent_case{1, {0.5, 0.25}, 2, 0.4},    descent_case{1, {-4.0, -4.0}, 2, 0.3}, descent_case{2, {1.5, 1.0}, 2, 0.2},
        descent_case{2, {-0.5, -1.5}, 2, 0.25},  descent_case{2, {0.25, 2.0}, 2, 0.5},  descent_case{2, {-1.75, 0.5}, 2, 0.1},
}};

// The cases from which the descent is stopped at descent_limit_cap steps.
constexpr std::array<std::size_t, 11> descent_limit_cases = {0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11};

constexpr std::uint32_t descent_iteration_cap = 550;
constexpr std::uint32_t descent_limit_cap     = 3;

// The greatest entry of the gradient at the answer, twice; the stationary test of every problem.
constexpr double stationary_magnitude = 1e-12;
constexpr double stationary_linear    = 1e-12;

constexpr pinned_bound stationary_bound{stationary_magnitude, stationary_linear, bound_standing::at_the_platform, bound_standing::at_the_platform};

// The greatest entry difference between an iterate and the step x - gamma grad f(x) from the one before it.
constexpr double descent_step_absolute = 1e-12;

}

#endif

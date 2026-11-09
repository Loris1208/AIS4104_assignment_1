#include "root_finding.h"

#include <praxis/optimization/registered.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace ais4104 {
namespace {

constexpr double first_link  = 1.0;
constexpr double second_link = 0.5;
constexpr double reached_x   = 0.75;

double two_roots(double x)
{
    return 1.0 - x * x;
}

double two_roots_slope(double x)
{
    return -2.0 * x;
}

double cubic(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

double cycle(double x)
{
    return x * x * x - 2.0 * x + 2.0;
}

double cycle_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

double arctangent(double x)
{
    return std::atan(x);
}

double arctangent_slope(double x)
{
    return 1.0 / (1.0 + x * x);
}

double unit_circle(double x, double y)
{
    return 1.0 - x * x - y * y;
}

Eigen::Vector2d unit_circle_gradient(double x, double y)
{
    return Eigen::Vector2d(-2.0 * x, -2.0 * y);
}

double two_link(double x, double y)
{
    return first_link * std::cos(x) + second_link * std::cos(x + y) - reached_x;
}

Eigen::Vector2d two_link_gradient(double x, double y)
{
    return Eigen::Vector2d(-first_link * std::sin(x) - second_link * std::sin(x + y), -second_link * std::sin(x + y));
}

}

praxis::optimization::registered_curve two_roots_curve()
{
    return praxis::optimization::registered_curve("1 - x^2", &two_roots, &two_roots_slope);
}

praxis::optimization::registered_curve cubic_curve()
{
    return praxis::optimization::registered_curve("x^3 - 2x - 5", &cubic, &cubic_slope);
}

praxis::optimization::registered_curve cycle_curve()
{
    return praxis::optimization::registered_curve("x^3 - 2x + 2", &cycle, &cycle_slope);
}

praxis::optimization::registered_curve arctangent_curve()
{
    return praxis::optimization::registered_curve("atan(x)", &arctangent, &arctangent_slope);
}

praxis::optimization::registered_surface unit_circle_surface()
{
    return praxis::optimization::registered_surface("1 - x^2 - y^2", &unit_circle, &unit_circle_gradient);
}

praxis::optimization::registered_surface two_link_surface()
{
    return praxis::optimization::registered_surface("1 cos(x) + 0.5 cos(x + y) - 0.75", &two_link, &two_link_gradient);
}

std::vector<praxis::optimization::registered_curve> root_finding_curves()
{
    return {two_roots_curve(), cubic_curve(), cycle_curve(), arctangent_curve()};
}

std::vector<praxis::optimization::registered_surface> root_finding_surfaces()
{
    return {unit_circle_surface(), two_link_surface()};
}

}

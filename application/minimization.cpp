#include "minimization.h"

#include <praxis/optimization/registered.h>

#include <Eigen/Core>

#include <vector>

namespace ais4104 {
namespace {

constexpr double bowl_stiffness = 10.0;

double parabola(double x)
{
    return (x - 1.0) * (x - 1.0);
}

double parabola_slope(double x)
{
    return 2.0 * (x - 1.0);
}

double parabola_curvature(double)
{
    return 2.0;
}

double quartic(double x)
{
    return x * x * x * x - 3.0 * x * x + x;
}

double quartic_slope(double x)
{
    return 4.0 * x * x * x - 6.0 * x + 1.0;
}

double quartic_curvature(double x)
{
    return 12.0 * x * x - 6.0;
}

double elongated_bowl(double x, double y)
{
    return (x * x + bowl_stiffness * y * y) / 2.0;
}

Eigen::Vector2d elongated_bowl_gradient(double x, double y)
{
    return Eigen::Vector2d(x, bowl_stiffness * y);
}

Eigen::Matrix2d elongated_bowl_hessian(double, double)
{
    return Eigen::Matrix2d{{1.0, 0.0}, {0.0, bowl_stiffness}};
}

// Nocedal & Wright, Numerical Optimization, 1st ed., eq. (2.23).
double rosenbrock(double x, double y)
{
    return 100.0 * (y - x * x) * (y - x * x) + (1.0 - x) * (1.0 - x);
}

Eigen::Vector2d rosenbrock_gradient(double x, double y)
{
    return Eigen::Vector2d(-400.0 * x * (y - x * x) - 2.0 * (1.0 - x), 200.0 * (y - x * x));
}

Eigen::Matrix2d rosenbrock_hessian(double x, double y)
{
    return Eigen::Matrix2d{{1200.0 * x * x - 400.0 * y + 2.0, -400.0 * x}, {-400.0 * x, 200.0}};
}

}

praxis::optimization::registered_curve parabola_curve()
{
    return praxis::optimization::registered_curve("(x - 1)^2", &parabola, &parabola_slope, &parabola_curvature);
}

praxis::optimization::registered_curve quartic_curve()
{
    return praxis::optimization::registered_curve("x^4 - 3x^2 + x", &quartic, &quartic_slope, &quartic_curvature);
}

praxis::optimization::registered_surface elongated_bowl_surface()
{
    return praxis::optimization::registered_surface("(x^2 + 10y^2) / 2", &elongated_bowl, &elongated_bowl_gradient, &elongated_bowl_hessian);
}

praxis::optimization::registered_surface rosenbrock_surface()
{
    return praxis::optimization::registered_surface("100(y - x^2)^2 + (1 - x)^2", &rosenbrock, &rosenbrock_gradient, &rosenbrock_hessian);
}

std::vector<praxis::optimization::registered_curve> minimization_curves()
{
    return {parabola_curve(), quartic_curve()};
}

std::vector<praxis::optimization::registered_surface> minimization_surfaces()
{
    return {elongated_bowl_surface(), rosenbrock_surface()};
}

}

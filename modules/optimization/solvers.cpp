//Assignment 3
#include "optimization/solvers.h"

#include "praxis/config/error.h"

#include <Eigen/LU>

#include <cmath>

namespace ais4104::optimization {

//TASK: T.1a
//REFERENCE: Algorithm and equation (6.6) page 228, MR pre-print 2019
praxis::expected<Eigen::VectorXd, praxis::refusal> newton_raphson(const praxis::optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations,
                                                                  std::vector<Eigen::VectorXd> &iterates)
{
    //append x0 to iterates
    //for each iteration:
    //    evaluate the error e(x) with p.error
    //    return x when p.converged holds for e(x)
    //    make the step
    //    append the new x to iterates
    //past max_iterations: return praxis::unexpected(praxis::refusal::no_solution)
    double n;
    double m;
    Eigen::VectorXd e;
    Eigen::MatrixXd J_pinv;
    Eigen::VectorXd x = x0;
    iterates.push_back(x0);
    for (int i = 0; i < max_iterations; i++) {
        auto err = p.error(x);
        if (!err) {
            return praxis::unexpected(err.error());
        }
        e = *err;
        auto jac = p.jacobian(x);
        if (!jac) {
            return praxis::unexpected(jac.error());
        }
        const Eigen::MatrixXd &J = *jac;
        m = J.rows();
        n = J.cols();
        if (n > m)
        {
            J_pinv = J.transpose() * (J * J.transpose()).inverse();
        }
        if (n < m)
        {
            J_pinv = (J.transpose() * J).inverse() * J.transpose();
        }
        if (m == n)
        {
            J_pinv = J.inverse();
        }
        if (p.converged(e))
        {
            return x;
        }
        x = x + J_pinv * e;
        iterates.push_back(x);
    }
    return praxis::unexpected(praxis::refusal::no_solution);
}

//TASK: T.1b
//REFERENCE: Section (2.6.3) page 36, doctoral thesis Skrede
praxis::expected<Eigen::VectorXd, praxis::refusal> gradient_descent(const praxis::optimization::problem &p, const Eigen::VectorXd &x0, double step_length,
                                                                    std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    //append x0 to iterates
    //for each iteration:
    //    evaluate the gradient of f at x with p.gradient
    //    return x when p.stationary holds for the gradient
    //    make the step
    //    append the new x to iterates
    //past max_iterations: return praxis::unexpected(praxis::refusal::no_solution)
    Eigen::VectorXd e;
    Eigen::VectorXd x = x0;
    iterates.push_back(x0);
    for (int i = 0; i < max_iterations; i++) {
        auto grad = p.gradient(x);
        if (!grad) {
            return praxis::unexpected(grad.error());
        }
        const Eigen::MatrixXd &g = *grad;
        if (p.stationary(g))
        {
            return x;
        }
        x = x - step_length * g;
        iterates.push_back(x);
    }
    return praxis::unexpected(praxis::refusal::no_solution);
}

}

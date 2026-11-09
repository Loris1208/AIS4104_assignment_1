#ifndef HPP_GUARD_AIS4104_OPTIMIZATION_SOLVERS_H
#define HPP_GUARD_AIS4104_OPTIMIZATION_SOLVERS_H

#include <praxis/optimization/problem.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <Eigen/Core>

#include <vector>
#include <cstdint>

namespace ais4104::optimization {

praxis::expected<Eigen::VectorXd, praxis::refusal> newton_raphson(const praxis::optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations,
                                                                  std::vector<Eigen::VectorXd> &iterates);

praxis::expected<Eigen::VectorXd, praxis::refusal> gradient_descent(const praxis::optimization::problem &p, const Eigen::VectorXd &x0, double step_length,
                                                                    std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates);

}

#endif

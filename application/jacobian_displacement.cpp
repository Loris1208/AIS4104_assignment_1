#include "jacobian_displacement.h"

#include <Eigen/SVD>
#include <Eigen/Core>

#include <algorithm>

namespace ais4104 {
namespace {

constexpr double singularity_threshold = 1.0e-2;

constexpr double lambda_max = 4.0e-2;

// The longest interval one tick integrates, in seconds.
constexpr double jog_step_seconds = 2.0e-2;

constexpr float twist_slider_extent = 1.0f;

// The column count is the one shape a binding decides at run time: a Jacobian carries six rows by
// its own type, and the limits are sized by the chain the composition refused to build otherwise.
// Every size agrees when every size is zero, so the joint count is held strictly positive too.
bool sized_to_the_joints(const praxis::manipulator::jacobian &of, const praxis::manipulator::joint_vector &at, const praxis::manipulator::joint_limits &within)
{
    return at.size() > 0 && of.cols() == at.size() && within.velocity.size() == at.size() && within.lower_position.size() == at.size() && within.upper_position.size() == at.size();
}

// Damped least squares with the damping adapted to the smallest singular value, after Nakamura,
// Advanced Robotics: Redundancy and Optimization, 1991, chapter 11.
Eigen::VectorXd joint_rates(const praxis::manipulator::jacobian &of, const praxis::twist &asked)
{
    const Eigen::JacobiSVD<Eigen::MatrixXd> decomposed(of, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd sigma = decomposed.singularValues();
    const double least          = sigma[sigma.size() - 1];

    // The damping is strictly positive exactly where the least singular value is under the
    // threshold, so no axis below carries a vanishing denominator.
    const double ratio   = least / singularity_threshold;
    const double damping = least < singularity_threshold ? lambda_max * lambda_max * (1.0 - ratio * ratio) : 0.0;

    Eigen::VectorXd inverted(sigma.size());
    for(Eigen::Index axis = 0; axis < sigma.size(); ++axis)
        inverted[axis] = sigma[axis] / (sigma[axis] * sigma[axis] + damping);

    return decomposed.matrixV() * inverted.asDiagonal() * decomposed.matrixU().transpose() * asked;
}

// One scalar over the whole vector: scaling every rate by the same factor scales the achieved twist
// by that factor, so a bounded step is slower than the one asked for and points the same way.
Eigen::VectorXd held_inside_the_limits(const Eigen::VectorXd &rates, const praxis::manipulator::joint_limits &within)
{
    const double worst = rates.cwiseAbs().maxCoeff();
    const double bound = within.velocity.minCoeff();
    if(worst <= bound)
        return rates;

    return (bound / worst) * rates;
}

double integrated_over(praxis::scheduler::step_delta elapsed)
{
    const double seconds = elapsed.value.count();

    return seconds > 0.0 ? std::min(seconds, jog_step_seconds) : 0.0;
}

}

const float twist_slider_bound = twist_slider_extent;

praxis::manipulator::jacobian referenced_to(const praxis::manipulator::jacobian &of, const Eigen::Vector3d &point)
{
    Eigen::Matrix3d crossed;
    crossed << 0.0, -point.z(), point.y(), point.z(), 0.0, -point.x(), -point.y(), point.x(), 0.0;

    praxis::manipulator::jacobian referenced = of;
    referenced.bottomRows(3) -= crossed * of.topRows(3);

    return referenced;
}

praxis::expected<praxis::manipulator::joint_vector, praxis::refusal> jogged_configuration(const praxis::manipulator::jacobian &of, const praxis::twist &asked,
                                                                                          const praxis::manipulator::joint_vector &at, const praxis::manipulator::joint_limits &within,
                                                                                          praxis::scheduler::step_delta elapsed)
{
    if(!sized_to_the_joints(of, at, within) || !of.allFinite() || !within.velocity.allFinite() || within.velocity.minCoeff() <= 0.0)
        return praxis::unexpected(praxis::refusal::unsupported_input);

    const praxis::manipulator::joint_vector commanded = at + integrated_over(elapsed) * held_inside_the_limits(joint_rates(of, asked), within);
    if(!commanded.allFinite())
        return praxis::unexpected(praxis::refusal::degenerate);

    return commanded.cwiseMax(within.lower_position).cwiseMin(within.upper_position);
}

}

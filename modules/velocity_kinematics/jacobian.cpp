//Assignment 2
#include "velocity_kinematics/jacobian.h"

#include <span>
#include <vector>
#include <cstddef>

namespace ais4104::manipulator {

//TASK: 4a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 5.1.1, eq. (5.11).
praxis::expected<praxis::manipulator::jacobian, praxis::refusal> space_jacobian(const praxis::rigid_motion::screw_ops &screw, std::span<const Eigen::Vector6d> space_screws,
                                                                                const Eigen::VectorXd &theta)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 4b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 5.1.2, eq. (5.18).
praxis::expected<praxis::manipulator::jacobian, praxis::refusal> body_jacobian(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                               const praxis::manipulator::forward_kinematics_ops &forward, const Eigen::Matrix4d &m,
                                                                               std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}

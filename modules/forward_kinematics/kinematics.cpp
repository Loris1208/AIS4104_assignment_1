//Assignment 2
#include "forward_kinematics/kinematics.h"

#include <span>
#include <vector>

namespace ais4104::manipulator {

//TASK: 2a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1, eq. (4.14).
praxis::expected<Eigen::Matrix4d, praxis::refusal> forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const Eigen::Matrix4d &m,
                                                                      std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 2b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.3, B_i = [Ad_{M^-1}] S_i below eq. (4.16), p. 147.
praxis::expected<std::vector<Eigen::Vector6d>, praxis::refusal> body_screws_from_space(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                                       const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

//TASK: 2c
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.3, eq. (4.16), p. 147.
praxis::expected<Eigen::Matrix4d, praxis::refusal> body_forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                           const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    return praxis::unexpected(praxis::refusal::not_implemented);
}

}

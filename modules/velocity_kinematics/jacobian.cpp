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
    int n = space_screws.size();
    praxis::manipulator::jacobian Js;
    Js.resize(6, n);
    praxis::transform t = Eigen::Matrix4d::Identity();
    double epsilon = 1e-9;
    for (int i = 0; i < n; i++)
    {
        if ((abs(space_screws[i].head<3>().norm() - 1.0) < epsilon) or (space_screws[i].head<3>().norm() < epsilon and (abs(space_screws[i].head<3>().norm() - 1.0)) < epsilon) or theta.size() <= n)
        {
            for (int j = 0; j < i+1; j++)
            {
                t =  t * screw.matrix_exponential_screw(space_screws[j], theta(j));
            }
            Js.col(i) = (screw.adjoint_matrix_from_transform(t).value() * space_screws[i]);
            t = Eigen::Matrix4d::Identity();
        }
        else
        {
            return praxis::unexpected(praxis::refusal::degenerate);
        }
    }
    return Js;
}

//TASK: 4b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 5.1.2, eq. (5.18).
praxis::expected<praxis::manipulator::jacobian, praxis::refusal> body_jacobian(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                               const praxis::manipulator::forward_kinematics_ops &forward, const Eigen::Matrix4d &m,
                                                                               std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    int n = space_screws.size();
    praxis::manipulator::jacobian Jt;
    Jt.resize(6, n);
    praxis::transform t = Eigen::Matrix4d::Identity();
    double epsilon = 1e-9;
    std::vector<Eigen::Vector6d> b = forward.body_screws_from_space(screw, frames, m, space_screws).value();
    for (int i = 0; i < n; i++)
    {
        if ((abs(space_screws[i].head<3>().norm() - 1.0) < epsilon) or (space_screws[i].head<3>().norm() < epsilon and (abs(space_screws[i].head<3>().norm() - 1.0)) < epsilon) or theta.size() <= n)
        {
            for (int j = i; j < n; j++)
            {
                t =  t * screw.matrix_exponential_screw(b[j], theta(j));
            }
            Jt.col(i) = (screw.adjoint_matrix_from_transform(frames.inverse(t)).value() * b[i]);
            t = Eigen::Matrix4d::Identity();
        }
        else
        {
            return praxis::unexpected(praxis::refusal::degenerate);
        }
    }
    return Jt;
}

}

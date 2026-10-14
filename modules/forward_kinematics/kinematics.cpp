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
    praxis::transform t = m;
    int n = space_screws.size();
    for (int i = 1; i < n+1; i++)
    {
    t = screw.matrix_exponential_screw(space_screws[n-i], theta(n-i))*t;
    }
    return t;
}

//TASK: 2b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.3, B_i = [Ad_{M^-1}] S_i below eq. (4.16), p. 147.
praxis::expected<std::vector<Eigen::Vector6d>, praxis::refusal> body_screws_from_space(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                                       const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws)
{
    Eigen::Matrix4d minv;
    Eigen::Vector6d bi;
    std::vector<Eigen::Vector6d> b;
    praxis::rotation ri;
    Eigen::Vector3d pi;
    int n = space_screws.size();
    minv = frames.inverse(m);
    ri = frames.rotation_matrix_from_transform(minv);
    pi = minv.block<3,1>(0,3);
    for (int i = 0; i < n; i++)
    {
        bi = screw.adjoint_matrix_from_rotation_position(ri, pi).value() * space_screws[i];
        b.push_back(bi);
    }
    return b;
}

//TASK: 2c
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.3, eq. (4.16), p. 147.
praxis::expected<Eigen::Matrix4d, praxis::refusal> body_forward_kinematics(const praxis::rigid_motion::screw_ops &screw, const praxis::rigid_motion::frame_ops &frames,
                                                                           const Eigen::Matrix4d &m, std::span<const Eigen::Vector6d> space_screws, const Eigen::VectorXd &theta)
{
    praxis::transform t = m;
    Eigen::Vector6d bi;
    int n = space_screws.size();
    for (int i = 0; i < n; i++)
    {
        bi = body_screws_from_space(screw, frames, m, space_screws).value()[i];
        t = t*screw.matrix_exponential_screw(bi, theta(i));
    }
    return t;
}

}

//Assignment 3
#include "trajectory_generation/path.h"

#include <utility>

namespace ais4104::trajectory {

//TASK: T.3a
//REFERENCE: Equation (9.3) page 328, MR pre-print 2019
praxis::expected<Eigen::VectorXd, praxis::refusal> joint_straight_line(const Eigen::VectorXd &start, const Eigen::VectorXd &end, double s)
{
    Eigen::VectorXd theta;
    if (start.size() != end.size() and (s > 1 or s < 0))
    {
        return praxis::unexpected(praxis::refusal::unsupported_input);
    }
    theta = start + s *(end-start);
    return theta;
}

//TASK: T.3b
//REFERENCE: Equation (9.6) page 330, MR pre-print 2019
praxis::expected<Eigen::Matrix4d, praxis::refusal> screw(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                         const Eigen::Matrix4d &end, double s)
{
    Eigen::Matrix4d x;
    Eigen::Vector6d S;
    Eigen::Vector3d w ,v;
    double theta;
    std::pair<Eigen::Vector6d, double> result = motion.matrix_logarithm_se3(frames.inverse(start) * end).value();
    S = result.first;
    theta = result.second;
    w = S.head<3>();
    v = S.tail<3>();
    x = start * motion.matrix_exponential_se3(s * w, s * v, theta);
    return x;
}

//TASK: T.3c
//REFERENCE: Equations (9.7) and (9.8)  page 330, MR pre-print 2019
praxis::expected<Eigen::Matrix4d, praxis::refusal> decoupled(const praxis::rigid_motion::screw_ops &motion, const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &start,
                                                             const Eigen::Matrix4d &end, double s)
{
    Eigen::Matrix3d r, r_start, r_end;
    Eigen::Vector3d p, p_start, p_end, w;
    Eigen::Matrix4d x;
    double theta;
    r_start = frames.rotation_matrix_from_transform(start);
    r_end = frames.rotation_matrix_from_transform(end);
    p_start = start.block<3,1>(0,3);
    p_end = end.block<3,1>(0,3);
    p = p_start + s * (p_end - p_start);
    std::pair<Eigen::Vector3d, double> result = motion.matrix_logarithm_so3((r_start.transpose() * r_end)).value();
    w = result.first;
    theta = result.second;
    r = r_start * motion.matrix_exponential_so3(s*w, theta);
    x = frames.transformation_matrix_from_rotation_position(r,p);
    return x;
}

}

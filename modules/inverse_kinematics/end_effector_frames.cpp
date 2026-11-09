//Assignment 3
#include "inverse_kinematics/end_effector_frames.h"

namespace ais4104::manipulator {

//TASK: T.2a
praxis::expected<Eigen::Matrix4d, praxis::refusal> tool_pose_from_flange_pose(const Eigen::Matrix4d &flange_pose, const Eigen::Matrix4d &tool_offset)
{
    Eigen::Matrix4d tool_pose  = flange_pose * tool_offset;
    return tool_pose;
}

//TASK: T.2b
praxis::expected<Eigen::Matrix4d, praxis::refusal> flange_pose_from_tool_pose(const praxis::rigid_motion::frame_ops &frames, const Eigen::Matrix4d &tool_pose,
                                                                              const Eigen::Matrix4d &tool_offset)
{
    Eigen::Matrix4d flange_pose = tool_pose * frames.inverse(tool_offset);
    return flange_pose;
}

//TASK: T.2c
praxis::expected<Eigen::Vector3d, praxis::refusal> position_from_pose(const Eigen::Matrix4d &pose)
{
    Eigen::Vector3d p = pose.block<3,1>(0,3);
    return p;
}

//TASK: T.2c
praxis::expected<Eigen::Matrix3d, praxis::refusal> orientation_from_pose(const Eigen::Matrix4d &pose)
{
    Eigen::Matrix3d r = pose.block<3,3>(0,0);
    return r;
}

//TASK: T.2d
praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_flange_pose(const praxis::manipulator::kinematics &solver, const Eigen::Matrix4d &flange_pose, const Eigen::VectorXd &j0)
{
    Eigen::VectorXd theta;
    if (j0.size() != solver.joint_count())
    {
        return praxis::unexpected(praxis::refusal::unsupported_input);
    }
    theta = solver.ik_solve(flange_pose, j0, praxis::manipulator::solver_parameters()).value();
    return theta;
}

//TASK: T.2e
praxis::expected<Eigen::VectorXd, praxis::refusal> ik_solve_pose(const praxis::manipulator::kinematics &solver, const praxis::rigid_motion::frame_ops &frames,
                                                                 const Eigen::Matrix4d &tool_pose, const Eigen::VectorXd &j0, const Eigen::Matrix4d &tool_offset)
{
    Eigen::Matrix4d flange_pose;
    Eigen::VectorXd theta;
    if (j0.size() != solver.joint_count())
    {
        return praxis::unexpected(praxis::refusal::unsupported_input);
    }
    flange_pose = flange_pose_from_tool_pose(frames, tool_pose, tool_offset).value();
    theta = solver.ik_solve(flange_pose, j0, praxis::manipulator::solver_parameters()).value();
    return theta;
}

}

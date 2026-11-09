#include "inverse_kinematics/capabilities.h"
#include "inverse_kinematics/end_effector_frames.h"

namespace ais4104::manipulator {

namespace {

constexpr praxis::manipulator::robot_ops bound_robot{
        .tool_pose_from_flange_pose = &tool_pose_from_flange_pose,
        .flange_pose_from_tool_pose = &flange_pose_from_tool_pose,
        .position_from_pose         = &position_from_pose,
        .orientation_from_pose      = &orientation_from_pose,
        .ik_solve_pose              = &ik_solve_pose,
        .ik_solve_flange_pose       = &ik_solve_flange_pose,
};

}

praxis::manipulator::robot_ops end_effector_frames()
{
    return bound_robot;
}

}

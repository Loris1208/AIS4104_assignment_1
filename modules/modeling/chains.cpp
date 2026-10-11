//Assignment 2
#include "modeling/chains.h"

#include <praxis/rigid_motion/angles.h>

namespace ais4104::modeling {

//TASK: 1a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1.
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_kr6r900sixx_chain(const praxis::rigid_motion::screw_ops &screw)
{



    return {Eigen::Matrix4d::Zero(), {}};
}

//TASK: 1b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1.
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_lbr_med14_r820_chain(const praxis::rigid_motion::screw_ops &screw)
{
    return {Eigen::Matrix4d::Zero(), {}};
}

//TASK: 3a
Eigen::Matrix4d hardcoded_kr6r900sixx_curved_rod_tool_offset(const praxis::rigid_motion::frame_ops &frames)
{
    return Eigen::Matrix4d::Zero();
}

//TASK: 3b
Eigen::Matrix4d hardcoded_kr6r900sixx_humerus_tool_offset(const praxis::rigid_motion::frame_ops &frames)
{
    return Eigen::Matrix4d::Zero();
}

}

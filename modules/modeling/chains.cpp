//Assignment 2
#include "modeling/chains.h"

#include <praxis/rigid_motion/angles.h>

namespace ais4104::modeling {

//TASK: 1a
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1.
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_kr6r900sixx_chain(const praxis::rigid_motion::screw_ops &screw)
{
    Eigen::Matrix4d m;
    Eigen::Vector6d s1, s2, s3, s4, s5, s6;
    Eigen::Vector3d w1, w2, w3, w4, w5, w6;
    Eigen::Vector3d q1, q2, q3, q4, q5, q6;
    Eigen::Vector3d v1, v2, v3, v4, v5, v6;
    m << 0, 0, 1, 0.980,
         0, 1, 0, 0,
         -1, 0, 0, 0.435,
         0, 0, 0, 1;
    w1 = {0,0,-1};
    w2 = {0, 1, 0};
    w3 = {0,1,0};
    w4 = {-1, 0, 0};
    w5 = {0,1,0};
    w6 = {-1, 0, 0};
    q1 = {0,0,0};
    q2 = {0.025, 0, 0.4};
    q3 = {0.480,0,0.4};
    q4 = {0.9, 0, 0.435};
    q5 = {0.9,0,0.435};
    q6 = {0.980, 0, 0.435};
    std::vector<Eigen::Vector3d> v = {v1,v2, v3, v4, v5,v6};
    std::vector<Eigen::Vector3d> w = {w1,w2, w3, w4, w5,w6};
    std::vector<Eigen::Vector3d> q = {q1, q2, q3, q4, q5, q6};
    std::vector<Eigen::Vector6d> s = {s1, s2, s3, s4, s5, s6};

    for (int i = 0; i < 6; i++)
    {
        v[i] = screw.skew_symmetric(w[i])*(-q[i]);
        s[i] = screw.screw_axis_from_angular_linear(w[i],v[i]);
    }
    return {m, {s[0], s[1], s[2], s[3], s[4], s[5]}};
}

//TASK: 1b
//REFERENCE: Lynch & Park, Modern Robotics, Ch. 4.1.1.
std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>> hardcoded_lbr_med14_r820_chain(const praxis::rigid_motion::screw_ops &screw)
{   Eigen::Matrix4d m;
    Eigen::Vector6d s1, s2, s3, s4, s5, s6, s7;
    Eigen::Vector3d w1, w2, w3, w4, w5, w6, w7;
    Eigen::Vector3d q1, q2, q3, q4, q5, q6, q7;
    Eigen::Vector3d v1, v2, v3, v4, v5, v6, v7;
    m << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 1.3055,
        0, 0, 0, 1;
    w1 = {0,0,1};
    w2 = {0, 1, 0};
    w3 = {0,0,1};
    w4 = {0, -1, 0};
    w5 = {0,0,1};
    w6 = {0, 1, 0};
    w7 = {0,0,1};
    q1 = {0,0,0};
    q2 = {0, 0, 0.3595};
    q3 = {0,0,0.3595};
    q4 = {0, 0, 0.7795};
    q5 = {0,0,0.9795};
    q6 = {0, 0, 1.1795};
    q7 = {0,0,1.3055};
    std::vector<Eigen::Vector3d> v = {v1,v2, v3, v4, v5,v6, v7};
    std::vector<Eigen::Vector3d> w = {w1,w2, w3, w4, w5,w6, w7};
    std::vector<Eigen::Vector3d> q = {q1, q2, q3, q4, q5, q6, q7};
    std::vector<Eigen::Vector6d> s = {s1, s2, s3, s4, s5, s6, s7};

    for (int i = 0; i < 7; i++)
    {
        v[i] = screw.skew_symmetric(w[i])*(-q[i]);
        s[i] = screw.screw_axis_from_angular_linear(w[i],v[i]);
    }
    return {m, {s[0], s[1], s[2], s[3], s[4], s[5], s[6]}};
}

//TASK: 3a
//REFERENCE:
Eigen::Matrix4d hardcoded_kr6r900sixx_curved_rod_tool_offset(const praxis::rigid_motion::frame_ops &frames)
{
    Eigen::Matrix4d tft;
    Eigen::Matrix3d r;
    Eigen::Vector3d p;
    r = frames.rotate_x(0)*frames.rotate_y(std::numbers::pi/2)*frames.rotate_z(0);
    p = {0.082, 0 , 0.132};
    tft = frames.transformation_matrix_from_rotation_position(r,p);
    return tft;
}

//TASK: 3b
Eigen::Matrix4d hardcoded_kr6r900sixx_humerus_tool_offset(const praxis::rigid_motion::frame_ops &frames)
{
    Eigen::Matrix4d tft;
    Eigen::Matrix3d r;
    Eigen::Vector3d p;
    r = frames.rotate_x(std::numbers::pi/2)*frames.rotate_y(-7*std::numbers::pi/12)*frames.rotate_z(0);
    p = {-0.1519, -0.0193 , 0.0421};
    tft = frames.transformation_matrix_from_rotation_position(r,p);
    return tft;
}

}

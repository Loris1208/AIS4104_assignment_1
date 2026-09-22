#include "rigid_motion/frame.h"
#include "rigid_motion/screw.h"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <numbers>

namespace ais4104::rigid_motion {

//TASK: 1b
//REFERENCE: Equation (3.30) page 75, MR pre-print 2019
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m_skew_symm =  Eigen::Matrix3d::Zero();
    m_skew_symm << 0.0, -v[2], v[1],v[2],0.0, -v[0],-v[1],v[0],0;
    return m_skew_symm;
}

//TASK: 1c
//REFERENCE: Equation (3.30) page 75, MR pre-print 2019
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
    Eigen::Vector3d v_from_skew_sym = {m(2,1),m(0,2),m(1,0)};
    return v_from_skew_sym;
}

//TASK: 3b
//REFERENCE: Frist formula in section 3.3.2.2 page 101 and formula S*theta=v page 102, MR pre-print 2019
praxis::expected<Eigen::Vector6d, praxis::refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    praxis::screw_axis screw = Eigen::Vector6d::Zero();
    if (s == Eigen::Vector3d::Zero())
    {
        return praxis::unexpected(praxis::refusal::degenerate);
    }
    screw.head<3>() = s;
    screw.tail<3>() = -skew_symmetric(s)*q+h*s;
    return screw;
}

//TASK: 3c
//REFERENCE: Definition (3.24) page 102, MR pre-print 2019
Eigen::Vector6d screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
   praxis::screw_axis screw;
    if (w.norm() == 0)
    {
        if (v.norm()==0)
        {
           screw =  Eigen::Vector6d::Zero();
        }
        else
        {
            screw.head<3>() = w/v.norm();
            screw.tail<3>() = v/v.norm();
        }
    }
    else
    {
        screw.head<3>() = w/w.norm();
        screw.tail<3>() = v/w.norm();
    }
   return screw;
}

//TASK: 3d
//REFERENCE: Fist formula in section 3.3.2.2 page 101, MR pre-print 2019
Eigen::Vector6d twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    praxis::twist twist;
    twist.head<3>() = w;
    twist.tail<3>() = v;
    return twist;
}

//TASK: 3e
//REFERENCE: Frist formula in section 3.3.2.2 page 101, MR pre-print 2019
praxis::expected<Eigen::Vector6d, praxis::refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    praxis::screw_axis screw;
    praxis::twist twist;
    screw = screw_axis_from_point_direction_pitch(q,s,h).value();
    twist = screw * angular_velocity;
    return twist;
}

//TASK: 3f
//REFERENCE: Definition (3.20) page 98, MR pre-print 2019
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_rotation_position(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix6d adt = Eigen::Matrix6d::Zero();
    double epsilon = 1e-9; //chosen at random
    if (std::abs(r.determinant() - 1) < epsilon and (r.transpose()*r - Eigen::Matrix3d::Identity()).norm() < epsilon)
    {
        adt.block<3,3>(0,0) = r;
        adt.block<3,3>(3,3) = r;
        adt.block<3,3>(3,0) = skew_symmetric(p) * r;
    }
    else
    {
        return praxis::unexpected(praxis::refusal::degenerate);
    }
    return adt;
}

//TASK: 3g
//REFERENCE: Definition (3.20) page 98 and equation (3.68) page 95, MR pre-print 2019
praxis::expected<Eigen::Matrix6d, praxis::refusal> adjoint_matrix_from_transform(const Eigen::Matrix4d &tf)
{
    praxis::rotation r = rotation_matrix_from_transform(tf);
    Eigen::Vector3d p;
    Eigen::Matrix6d adt;
    p = tf.block<3,1>(0,3);
    adt = adjoint_matrix_from_rotation_position(r,p).value();
    return adt;
}

//TASK: 3h
//REFERENCE: Definition (3.20) page 98, MR pre-print 2019
praxis::expected<Eigen::Vector6d, praxis::refusal> adjoint_map(const Eigen::Vector6d &t, const Eigen::Matrix4d &tf)
{
    Eigen::Vector6d t_adjoint = adjoint_matrix_from_transform(tf).value()*t;
    return t_adjoint;
}

//TASK: 3i
//REFERENCE: Equation (3.71) page 96, MR pre-print 2019
Eigen::Matrix4d twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Zero();
    t_matrix.block<3,3>(0,0) = skew_symmetric(w);
    t_matrix.block<3,1>(0,3) = v;
    return t_matrix;
}

//TASK: 3j
//REFERENCE: Equations (3.70) and (3.71) page 96, MR pre-print 2019
Eigen::Matrix4d twist_matrix_from_twist(const Eigen::Vector6d &t)
{
    Eigen::Vector3d w = t.head<3>();
    Eigen::Vector3d v = t.tail<3>();
    Eigen::Matrix4d t_matrix = twist_matrix_from_angular_linear(w,v);
    return t_matrix;
}

//TASK: 3k
//REFERENCE: Equation (3.51) page 82, MR pre-print 2019
Eigen::Matrix3d matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians)
{
    praxis::rotation r = rotation_matrix_from_axis_angle(w, theta_radians);
    return r;
}

//TASK: 3l
//REFERENCE: Proposition (3.25) page 103-104, MR pre-print 2019
Eigen::Matrix4d matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians)
{
    Eigen::Matrix4d matrix_exp = Eigen::Matrix4d::Identity();
    if (w == Eigen::Vector3d::Zero())
    {
        Eigen::Vector3d v_norm = v/v.norm();
        matrix_exp.block<3,1>(0,3) = v_norm*theta_radians;
    }
    else
    {
        Eigen::Vector3d v_norm = v/w.norm();
        Eigen::Vector3d w_norm = w/w.norm();
        matrix_exp.block<3,3>(0,0) =  matrix_exponential_so3(w_norm,theta_radians);
        Eigen::Matrix3d g =  (Eigen::Matrix3d::Identity() * theta_radians + (1 - std::cos(theta_radians)) * skew_symmetric(w_norm) + (theta_radians - std::sin(theta_radians)) * skew_symmetric(w_norm) * skew_symmetric(w_norm));
        matrix_exp.block<3,1>(0,3) = g*v_norm;
    }
    return matrix_exp;
}

//TASK: 3m
//REFERENCE: Definition (3.24) page 102 and proposition (3.25) page 103-104, MR pre-print 2019
Eigen::Matrix4d matrix_exponential_screw(const Eigen::Vector6d &s, double theta_radians)
{
    praxis::transform tf;
    Eigen::Vector3d w, v;
    w = s.head<3>();
    v = s.tail<3>();
    tf = matrix_exponential_se3(w, v, theta_radians);
    return tf;

}

//TASK: 3n
//REFERENCE: Algorithm page 85, MR pre-print 2019
praxis::expected<std::pair<Eigen::Vector3d, double>, praxis::refusal> matrix_logarithm_so3(const Eigen::Matrix3d &r)
{
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    double theta_radians;
    double epsilon = 1e-9;
    if ((r - Eigen::Matrix3d::Identity()).norm() < epsilon)
    {
        theta_radians = 0;
        praxis::refusal w = praxis::refusal::no_solution;
    }
    else if (r.trace() == -1)
    {
        theta_radians = std::numbers::pi;
        if (r(2,2) + 1.0 > epsilon)
        {
            w = (1.0/std::sqrt(2.0 * (1.0 + r(2,2)))) * Eigen::Vector3d (r(0,2),r(1,2),1 + r(2,2));
        }
        else if (r(1,1) + 1.0 > epsilon)
        {
            w = (1.0/std::sqrt(2.0 * (1.0 + r(1,1)))) * Eigen::Vector3d (r(0,1),1.0 + r(1,1),r(2,1));
        }
        else
        {
            w = (1.0/std::sqrt(2.0 * (1.0 + r(0,0)))) * Eigen::Vector3d (1.0 + r(0,0),r(1,0),r(2,0));
        }
    }
    else
    {
        theta_radians = std::acos(0.5*(r.trace() - 1));
        Eigen::Matrix3d skew_w = 1.0/(2.0 * std::sin(theta_radians))*(r - r.transpose());
        w = from_skew_symmetric(skew_w);
    }
    //return praxis::unexpected(praxis::refusal::not_implemented);
    return std::pair<Eigen::Vector3d, double>(w, theta_radians); //problem here with praxis::refusal w = praxis::refusal::no_solution;
}

//TASK: 3o
//REFERENCE: Algorithm in section 3.3.3.2 page 104, MR pre-print 2019
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3_rp(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    double epsilon = 1e-9;
    double theta_radians;
    Eigen::Vector3d w, v;
    praxis::screw_axis s;
    if ((r-Eigen::Matrix3d::Identity()).norm() < epsilon)
    {
        w =  Eigen::Vector3d::Zero();
        v = p/p.norm();
        theta_radians = p.norm();
    }else
    {
        std::pair<Eigen::Vector3d, double> result= matrix_logarithm_so3(r).value();
        w = result.first;
        theta_radians = result.second;
        v = ((1.0/theta_radians) * Eigen::Matrix3d::Identity() - skew_symmetric(w)/2.0 + (1.0/theta_radians - 0.5 * 1.0/std::tan(theta_radians/2.0)) * skew_symmetric(w) * skew_symmetric(w)) * p;
    }
    s.head<3>() = w;
    s.tail<3>() = v;
    return std::pair<Eigen::Vector6d, double>(s, theta_radians);
}

//TASK: 3p
//REFERENCE: Definition (3.13) page 87 and algorithm in section 3.3.3.2 page 104, MR pre-print 2019
praxis::expected<std::pair<Eigen::Vector6d, double>, praxis::refusal> matrix_logarithm_se3(const Eigen::Matrix4d &tf)
{
    praxis::screw_axis s;
    double theta_radians;
    praxis::rotation r = rotation_matrix_from_transform(tf);
    Eigen::Vector3d p = tf.block<3,1>(0,3);
    std::pair<Eigen::Vector6d, double> result = matrix_logarithm_se3_rp(r,p).value();
    s = result.first;
    theta_radians = result.second;
    return std::pair<Eigen::Vector6d, double>(s, theta_radians);
}

}
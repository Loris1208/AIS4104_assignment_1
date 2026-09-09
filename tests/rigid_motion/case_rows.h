#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_CASE_ROWS_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_CASE_ROWS_H

#include <praxis/extension/refusal.h>

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>

namespace ais4104 {

// How many ordinary cases a slot is graded on. A row holds its input whole rather than the scalars
// it was drawn from: a reconstruction inside the test would have to be bit-identical to the
// composition path the input came off, and it is not. Matrices are stored row by row.
constexpr std::size_t typical_cases_per_slot = 8;

// One type per input and output shape, so two slots taking the same inputs and answering the same
// shape share one. The name spells the shape rather than the slot for that reason.

struct vector_to_matrix3_case
{
    std::array<double, 3> vector;
    std::array<double, 9> expected;
};

struct matrix3_to_vector_case
{
    std::array<double, 9> matrix;
    std::array<double, 3> expected;
};

struct rotation_position_to_adjoint_case
{
    std::array<double, 9> rotation;
    std::array<double, 3> position_metres;
    std::array<double, 36> expected;
};

struct transform_to_adjoint_case
{
    std::array<double, 16> transform;
    std::array<double, 36> expected;
};

struct twist_transform_to_twist_case
{
    std::array<double, 6> twist;
    std::array<double, 16> transform;
    std::array<double, 6> expected;
};

struct angular_linear_to_six_case
{
    std::array<double, 3> angular;
    std::array<double, 3> linear;
    std::array<double, 6> expected;
};

struct screw_velocity_to_twist_case
{
    std::array<double, 3> point_metres;
    std::array<double, 3> direction;
    double pitch;
    double angular_velocity;
    std::array<double, 6> expected;
};

struct angular_linear_to_matrix4_case
{
    std::array<double, 3> angular;
    std::array<double, 3> linear;
    std::array<double, 16> expected;
};

struct twist_to_matrix4_case
{
    std::array<double, 6> twist;
    std::array<double, 16> expected;
};

struct point_direction_pitch_to_axis_case
{
    std::array<double, 3> point_metres;
    std::array<double, 3> direction;
    double pitch;
    std::array<double, 6> expected_axis;
};

struct direction_angle_to_rotation_case
{
    std::array<double, 3> direction;
    double radians;
    std::array<double, 9> expected;
};

struct angular_linear_angle_to_transform_case
{
    std::array<double, 3> angular;
    std::array<double, 3> linear;
    double radians;
    std::array<double, 16> expected;
};

struct axis_angle_to_transform_case
{
    std::array<double, 6> axis;
    double radians;
    std::array<double, 16> expected;
};

struct angle_to_matrix3_case
{
    double radians;
    std::array<double, 9> expected;
};

struct matrix3_order_to_matrix3_case
{
    std::array<double, 9> matrix;
    std::uint8_t axis_order;
    std::array<double, 9> expected;
};

struct matrix3_to_matrix3_case
{
    std::array<double, 9> matrix;
    std::array<double, 9> expected;
};

struct vector_order_to_matrix3_case
{
    std::array<double, 3> radians;
    std::uint8_t axis_order;
    std::array<double, 9> expected;
};

struct matrix4_to_matrix3_case
{
    std::array<double, 16> transform;
    std::array<double, 9> expected;
};

struct vector_to_matrix4_case
{
    std::array<double, 3> position_metres;
    std::array<double, 16> expected;
};

struct matrix3_to_matrix4_case
{
    std::array<double, 9> matrix;
    std::array<double, 16> expected;
};

struct matrix3_vector_to_matrix4_case
{
    std::array<double, 9> matrix;
    std::array<double, 3> position_metres;
    std::array<double, 16> expected;
};

struct matrix4_to_matrix4_case
{
    std::array<double, 16> transform;
    std::array<double, 16> expected;
};

struct rotation_to_rotation_log_case
{
    std::array<double, 9> rotation;
    std::array<double, 3> expected_axis;
    double expected_radians;
};

struct rotation_position_to_pose_log_case
{
    std::array<double, 9> rotation;
    std::array<double, 3> position_metres;
    std::array<double, 6> expected_axis;
    double expected_radians;
};

struct transform_to_pose_log_case
{
    std::array<double, 16> transform;
    std::array<double, 6> expected_axis;
    double expected_radians;
};

// A refusal row carries no answer, so its shape is its input alone and two slots refusing the same
// input shape share one type.

struct rotation_refusal_case
{
    std::array<double, 9> rotation;
    praxis::refusal expected;
};

struct rotation_position_refusal_case
{
    std::array<double, 9> rotation;
    std::array<double, 3> position_metres;
    praxis::refusal expected;
};

struct point_direction_pitch_refusal_case
{
    std::array<double, 3> point_metres;
    std::array<double, 3> direction;
    double pitch;
    praxis::refusal expected;
};

// A row stores its matrices row by row, which is how they are printed and read; Eigen's own storage
// order is not what the table's shape means.
template<std::size_t Size>
Eigen::Matrix<double, static_cast<int>(Size), 1> vector_of(const std::array<double, Size> &row)
{
    return Eigen::Map<const Eigen::Matrix<double, static_cast<int>(Size), 1>>(row.data());
}

inline Eigen::Matrix3d matrix3_of(const std::array<double, 9> &row)
{
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(row.data());
}

inline Eigen::Matrix4d matrix4_of(const std::array<double, 16> &row)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(row.data());
}

inline Eigen::Matrix<double, 6, 6> adjoint_of(const std::array<double, 36> &row)
{
    return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(row.data());
}

}

#endif

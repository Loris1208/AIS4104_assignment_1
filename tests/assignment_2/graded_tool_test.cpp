#include "graded_rows.h"
#include "described_chains.h"
#include "manipulator_rows.h"

#include <praxis/rigid_motion/angles.h>

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/LU>
#include <Eigen/Core>

#include <cmath>
#include <format>
#include <string>
#include <string_view>

namespace ais4104 {
namespace {

struct tool_reading
{
    double orthonormality;
    double determinant;
    Eigen::RowVector4d bottom;
    bool homogeneous;
    praxis::evaluation::residual apart;
};

Eigen::Matrix4d reference_transform(const tool_row &row, std::string_view named)
{
    const auto described = described_scenario(row.document);
    {
        INFO((described ? std::string() : described.error()));
        REQUIRE(described.has_value());
    }

    const auto &frame               = reference_motions().frame;
    const auto &tool                = described->tool;
    const Eigen::Matrix4d reference = frame.transformation_matrix_from_rotation_position(
            frame.rotation_matrix_from_euler(tool.kinematics_euler_degrees.cast<double>() * praxis::radians_per_degree, tool.kinematics_euler_order),
            tool.kinematics_offset.cast<double>());
    INFO(std::format("{}: {} states no tool transformation to compare against", named, row.document));
    REQUIRE(!reference.isIdentity());

    return reference;
}

tool_reading tool_of(const Eigen::Matrix4d &supplied, const Eigen::Matrix4d &reference)
{
    const Eigen::Matrix3d rotation  = supplied.topLeftCorner<3, 3>();
    const Eigen::RowVector4d bottom = supplied.row(3);

    return tool_reading{(rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm(), rotation.determinant(), bottom, bottom == Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0),
                        praxis::evaluation::pose_residual(supplied, reference)};
}

void check_tool(std::string_view row, const tool_reading &read)
{
    {
        INFO(std::format("{}: the tool transformation is not a rigid transformation: |R^T R - I| {:.6e}, det R {:.6e}, bottom row ({}, {}, {}, {})", row, read.orthonormality,
                         read.determinant, read.bottom[0], read.bottom[1], read.bottom[2], read.bottom[3]));
        CHECK(read.orthonormality <= tool_rigidity);
        CHECK(read.determinant > 0.0);
        CHECK(read.homogeneous);
    }
    {
        INFO(std::format("{}: residual {:.6e} rad / {:.6e} m allowed {:.6e} / {:.6e}", row, read.apart.magnitude, read.apart.linear_error_metres, tool_rotation_radians,
                         tool_translation_meters));
        CHECK(std::isfinite(read.apart.magnitude));
        CHECK(read.apart.magnitude <= tool_rotation_radians);
        CHECK(std::isfinite(read.apart.linear_error_metres));
        CHECK(read.apart.linear_error_metres <= tool_translation_meters);
    }
}

}
}

TEST_CASE("every hand-derived tool transform is the one its document states", "[modeling][graded]")
{
    for(const ais4104::tool_row &row : ais4104::tool_rows)
    {
        const std::string named = std::format("task {} - {} tool transform", row.task, row.slot);

        DYNAMIC_SECTION(named)
        {
            const Eigen::Matrix4d reference = ais4104::reference_transform(row, named);
            ais4104::check_tool(named, ais4104::tool_of(row.derive(ais4104::reference_motions().frame), reference));
        }
    }
}

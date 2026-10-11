#include "graded_rows.h"
#include "described_chains.h"
#include "manipulator_rows.h"

#include "modeling/chains.h"

#include <praxis/manipulator/screw_chain.h>

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <format>
#include <string>
#include <cstddef>
#include <string_view>

namespace ais4104 {
namespace {

struct home_pose_reading
{
    std::size_t supplied;
    std::size_t described;
    double orthonormality;
    double determinant;
    Eigen::RowVector4d bottom;
    bool homogeneous;
    praxis::evaluation::residual apart;
};

home_pose_reading home_pose_of(const chain_pair &supplied, const praxis::manipulator::screw_chain &described)
{
    const Eigen::Matrix4d &home     = supplied.first;
    const Eigen::Matrix3d rotation  = home.topLeftCorner<3, 3>();
    const Eigen::RowVector4d bottom = home.row(3);

    return home_pose_reading{supplied.second.size(),
                             described.joint_count(),
                             (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm(),
                             rotation.determinant(),
                             bottom,
                             bottom == Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0),
                             praxis::evaluation::pose_residual(home, described.home)};
}

void check_home_pose(std::string_view row, const home_pose_reading &read)
{
    {
        INFO(std::format("{}: {} screw axes supplied, the description has {} joints", row, read.supplied, read.described));
        CHECK(read.supplied == read.described);
    }
    {
        INFO(std::format("{}: the home pose is not a rigid transformation: |R^T R - I| {:.6e}, det R {:.6e}, bottom row ({}, {}, {}, {})", row, read.orthonormality, read.determinant,
                         read.bottom[0], read.bottom[1], read.bottom[2], read.bottom[3]));
        CHECK(read.orthonormality <= home_rigidity);
        CHECK(read.determinant > 0.0);
        CHECK(read.homogeneous);
    }
    {
        INFO(std::format("{}: residual {:.6e} rad / {:.6e} m allowed {:.6e} / {:.6e}", row, read.apart.magnitude, read.apart.linear_error_metres, home_rotation_radians,
                         home_translation_meters));
        CHECK(std::isfinite(read.apart.magnitude));
        CHECK(read.apart.magnitude <= home_rotation_radians);
        CHECK(std::isfinite(read.apart.linear_error_metres));
        CHECK(read.apart.linear_error_metres <= home_translation_meters);
    }
}

}
}

TEST_CASE("every hand-derived chain states the home pose its description derives", "[modeling][graded]")
{
    for(const ais4104::chain_row &row : ais4104::chain_rows)
    {
        const std::string named = std::format("task {} - {} home pose", row.task, row.slot);

        DYNAMIC_SECTION(named)
        {
            const auto described = ais4104::described_chain(row.document);
            {
                INFO((described ? std::string() : described.error()));
                REQUIRE(described.has_value());
            }

            ais4104::check_home_pose(named, ais4104::home_pose_of(row.derive(ais4104::reference_motions().screw), described.value()));
        }
    }
}

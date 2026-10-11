#include "manipulator_rows.h"
#include "manipulator_cases.h"

#include "velocity_kinematics/capabilities.h"

#include <praxis/manipulator/kinematics.h>

#include <praxis/rigid_motion/types.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <vector>

namespace ais4104 {
namespace {

const praxis::manipulator::differential_kinematics_ops student_differential = manipulator::jacobians();

case_reading space_jacobian_of(const frozen_chain &chain, const pinned_bound &bound)
{
    const praxis::rigid_motion::screw_ops &screw = reference_motions().screw;
    const std::vector<praxis::screw_axis> screws = space_screws_of(chain);
    const Eigen::VectorXd theta                  = theta_of(chain);

    return jacobian_reading(student_differential.space_jacobian(screw, screws, theta), reference_manipulator().dk.space_jacobian(screw, screws, theta), bound);
}

case_reading body_jacobian_of(const frozen_chain &chain, const pinned_bound &bound)
{
    const praxis::rigid_motion::screw_ops &screw               = reference_motions().screw;
    const praxis::rigid_motion::frame_ops &frames              = reference_motions().frame;
    const praxis::manipulator::forward_kinematics_ops &forward = reference_manipulator().fk;
    const Eigen::Matrix4d home                                 = home_of(chain);
    const std::vector<praxis::screw_axis> screws               = space_screws_of(chain);
    const Eigen::VectorXd theta                                = theta_of(chain);

    return jacobian_reading(student_differential.body_jacobian(screw, frames, forward, home, screws, theta),
                            reference_manipulator().dk.body_jacobian(screw, frames, forward, home, screws, theta), bound);
}

constexpr std::array<manipulator_row, 2> jacobian_rows{{
        {"4a", "jacobian.space_jacobian", "dk.space_jacobian", space_jacobian_bound, &space_jacobian_of},
        {"4b", "jacobian.body_jacobian", "dk.body_jacobian", body_jacobian_bound, &body_jacobian_of},
}};

}
}

TEST_CASE("every graded velocity kinematics task answers what the reference answers", "[velocity_kinematics][graded]")
{
    for(const ais4104::manipulator_row &row : ais4104::jacobian_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row);
        }
    }
}

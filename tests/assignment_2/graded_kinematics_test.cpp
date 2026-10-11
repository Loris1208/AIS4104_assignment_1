#include "manipulator_rows.h"
#include "manipulator_cases.h"

#include "forward_kinematics/capabilities.h"

#include <praxis/manipulator/kinematics.h>

#include <praxis/rigid_motion/types.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <format>
#include <vector>

namespace ais4104 {
namespace {

const praxis::manipulator::forward_kinematics_ops student_forward = manipulator::forward_maps();

case_reading forward_kinematics_of(const frozen_chain &chain, const pinned_bound &bound)
{
    const praxis::rigid_motion::screw_ops &screw = reference_motions().screw;
    const Eigen::Matrix4d home                   = home_of(chain);
    const std::vector<praxis::screw_axis> screws = space_screws_of(chain);
    const Eigen::VectorXd theta                  = theta_of(chain);

    return pose_reading(student_forward.forward_kinematics(screw, home, screws, theta), reference_manipulator().fk.forward_kinematics(screw, home, screws, theta), bound);
}

case_reading body_screws_of(const frozen_chain &chain, const pinned_bound &bound)
{
    const praxis::rigid_motion::screw_ops &screw  = reference_motions().screw;
    const praxis::rigid_motion::frame_ops &frames = reference_motions().frame;
    const Eigen::Matrix4d home                    = home_of(chain);
    const std::vector<praxis::screw_axis> screws  = space_screws_of(chain);

    return screw_axes_reading(student_forward.body_screws_from_space(screw, frames, home, screws), reference_manipulator().fk.body_screws_from_space(screw, frames, home, screws),
                              bound);
}

case_reading body_forward_kinematics_of(const frozen_chain &chain, const pinned_bound &bound)
{
    const praxis::rigid_motion::screw_ops &screw  = reference_motions().screw;
    const praxis::rigid_motion::frame_ops &frames = reference_motions().frame;
    const Eigen::Matrix4d home                    = home_of(chain);
    const std::vector<praxis::screw_axis> screws  = space_screws_of(chain);
    const Eigen::VectorXd theta                   = theta_of(chain);

    case_reading read = pose_reading(student_forward.body_forward_kinematics(screw, frames, home, screws, theta),
                                     reference_manipulator().fk.body_forward_kinematics(screw, frames, home, screws, theta), bound);
    if(!passed(read) && !passed(body_screws_of(chain, body_screws_bound)))
        read.clause += "; 2b also disagrees on this case, and 2c built on it cannot agree until it does";

    return read;
}

constexpr std::array<manipulator_row, 3> kinematics_rows{{
        {"2a", "kinematics.forward_kinematics", "fk.forward_kinematics", forward_kinematics_bound, &forward_kinematics_of},
        {"2b", "kinematics.body_screws_from_space", "fk.body_screws_from_space", body_screws_bound, &body_screws_of},
        {"2c", "kinematics.body_forward_kinematics", "fk.body_forward_kinematics", body_forward_kinematics_bound, &body_forward_kinematics_of},
}};

}
}

TEST_CASE("every graded forward kinematics task answers what the reference answers", "[forward_kinematics][graded]")
{
    for(const ais4104::manipulator_row &row : ais4104::kinematics_rows)
    {
        DYNAMIC_SECTION(std::format("task {} - {}", row.task, row.slot))
        {
            ais4104::check_row(row);
        }
    }
}

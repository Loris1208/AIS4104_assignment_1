#include "graded_rows.h"
#include "described_chains.h"
#include "manipulator_rows.h"

#include <praxis/manipulator/screw_chain.h>
#include <praxis/manipulator/screw_chain_difference.h>

#include <praxis/rigid_motion/types.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace ais4104 {
namespace {

praxis::manipulator::screw_chain_difference joints_apart(const chain_pair &answered, const praxis::manipulator::screw_chain &described)
{
    const std::vector<praxis::manipulator::supplied_screw> supplied(answered.second.begin(), answered.second.end());

    return praxis::manipulator::supplied_chain_difference(described, answered.first, supplied);
}

void check_term(std::string_view row, std::string_view term, std::string_view unit, const std::optional<double> &value, double bound, bool required)
{
    if(!value.has_value())
    {
        if(required)
            FAIL_CHECK(std::format("{}: praxis left '{}' unanswered", row, term));
        return;
    }

    INFO(std::format("{}: {} {:.6e}{} allowed {:.6e}{}", row, term, *value, unit, bound, unit));
    CHECK(std::isfinite(*value));
    CHECK(*value <= bound);
}

void check_joint(std::string_view row, const praxis::manipulator::chain_joint_difference &joint, const praxis::screw_axis &described)
{
    using reading       = praxis::manipulator::chain_joint_reading;
    const bool measured = joint.read == reading::measured;
    const bool turns    = described.head<3>().norm() > praxis::angular_epsilon;
    {
        INFO(std::format("{}: no screw supplied", row));
        CHECK(joint.read != reading::not_supplied);
    }
    {
        INFO(std::format("{}: the supplied screw translates where the description's turns, or turns where it translates", row));
        CHECK(joint.read != reading::kinds_differed);
    }
    if(joint.read != reading::not_supplied && joint.read != reading::kinds_differed)
    {
        INFO(std::format("{}: praxis compared nothing on this joint", row));
        CHECK(measured);
    }
    check_term(row, "direction off by", " rad", joint.direction_radians, screw_direction_radians, measured);
    check_term(row, "length off unit by", "", joint.length, screw_length_defect, measured);
    check_term(row, "linear part off by", " m", joint.moment_metres, screw_moment_meters, measured && turns);
}

}
}

TEST_CASE("every hand-derived screw axis is the one its description derives", "[modeling][graded]")
{
    for(const ais4104::chain_row &row : ais4104::chain_rows)
    {
        const auto described = ais4104::described_chain(row.document);
        if(!described)
        {
            const std::string named = std::format("task {} - {} description", row.task, row.slot);

            DYNAMIC_SECTION(named)
            {
                FAIL_CHECK(std::format("{}: {}", named, described.error()));
            }
            continue;
        }

        for(std::size_t joint = 0; joint < described->joint_count(); ++joint)
        {
            const std::string named = std::format("task {} - {} joint {}", row.task, row.slot, joint + 1);

            DYNAMIC_SECTION(named)
            {
                const praxis::manipulator::screw_chain_difference apart = ais4104::joints_apart(row.derive(ais4104::reference_motions().screw), *described);

                ais4104::check_joint(named, apart.joints[joint], described->space_screws[joint]);
            }
        }
    }
}

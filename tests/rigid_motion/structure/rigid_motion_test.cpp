#include "slot_report.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/evaluation.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/slot_evaluation.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <string_view>

namespace {

constexpr std::size_t frame_slots        = 12;
constexpr std::size_t screw_slots        = 17;
constexpr std::size_t rigid_motion_slots = frame_slots + screw_slots;
constexpr std::size_t few_cases          = 1;
constexpr std::size_t many_cases         = 8;

void require_named_table(const std::array<praxis::evaluation::evaluation_view, 2> &compared)
{
    REQUIRE(compared[0].slots().size() == frame_slots);
    REQUIRE(compared[1].slots().size() == screw_slots);
    REQUIRE(compared[0].slots().size() + compared[1].slots().size() == rigid_motion_slots);

    for(const praxis::evaluation::evaluation_view &view : compared)
    {
        REQUIRE_FALSE(view.extension().empty());

        for(const praxis::evaluation::slot_evaluation &slot : view.slots())
            REQUIRE_FALSE(slot.name.empty());
    }
}

std::array<bool, frame_slots> frame_bindings_apart(const praxis::rigid_motion::frame_ops &composed, const praxis::rigid_motion::frame_ops &reference)
{
    return {composed.euler_from_rotation_matrix != reference.euler_from_rotation_matrix,
            composed.rotate_x != reference.rotate_x,
            composed.rotate_y != reference.rotate_y,
            composed.rotate_z != reference.rotate_z,
            composed.rotation_matrix_from_frame_axes != reference.rotation_matrix_from_frame_axes,
            composed.rotation_matrix_from_euler != reference.rotation_matrix_from_euler,
            composed.rotation_matrix_from_axis_angle != reference.rotation_matrix_from_axis_angle,
            composed.rotation_matrix_from_transform != reference.rotation_matrix_from_transform,
            composed.transformation_matrix_from_position != reference.transformation_matrix_from_position,
            composed.transformation_matrix_from_rotation != reference.transformation_matrix_from_rotation,
            composed.transformation_matrix_from_rotation_position != reference.transformation_matrix_from_rotation_position,
            composed.inverse != reference.inverse};
}

std::array<bool, screw_slots> screw_bindings_apart(const praxis::rigid_motion::screw_ops &composed, const praxis::rigid_motion::screw_ops &reference)
{
    return {composed.skew_symmetric != reference.skew_symmetric,
            composed.from_skew_symmetric != reference.from_skew_symmetric,
            composed.adjoint_matrix_from_rotation_position != reference.adjoint_matrix_from_rotation_position,
            composed.adjoint_matrix_from_transform != reference.adjoint_matrix_from_transform,
            composed.adjoint_map != reference.adjoint_map,
            composed.twist_from_angular_linear != reference.twist_from_angular_linear,
            composed.twist_from_screw != reference.twist_from_screw,
            composed.twist_matrix_from_angular_linear != reference.twist_matrix_from_angular_linear,
            composed.twist_matrix_from_twist != reference.twist_matrix_from_twist,
            composed.screw_axis_from_angular_linear != reference.screw_axis_from_angular_linear,
            composed.screw_axis_from_point_direction_pitch != reference.screw_axis_from_point_direction_pitch,
            composed.matrix_exponential_so3 != reference.matrix_exponential_so3,
            composed.matrix_exponential_se3 != reference.matrix_exponential_se3,
            composed.matrix_exponential_screw != reference.matrix_exponential_screw,
            composed.matrix_logarithm_so3 != reference.matrix_logarithm_so3,
            composed.matrix_logarithm_se3_rp != reference.matrix_logarithm_se3_rp,
            composed.matrix_logarithm_se3 != reference.matrix_logarithm_se3};
}

praxis::rigid_motion::capabilities partly_written()
{
    const praxis::rigid_motion::capabilities reference = praxis::rigid_motion::baseline();

    praxis::rigid_motion::capabilities mixed{};
    mixed.frame.rotate_x = reference.frame.rotate_x;
    mixed.frame.rotate_z = reference.frame.rotate_x;

    return mixed;
}

ais4104::slot_state classified_state(const std::vector<ais4104::classified_slot> &classified, std::string_view name)
{
    const auto found = std::find_if(classified.begin(), classified.end(), [name](const ais4104::classified_slot &one) { return one.name == name; });
    REQUIRE(found != classified.end());

    return found->state;
}

void require_state(const praxis::rigid_motion::capabilities &mixed, std::string_view name, ais4104::slot_state wanted)
{
    for(std::size_t cases : {few_cases, many_cases})
    {
        INFO(name << " at " << cases << " case(s) per slot");
        REQUIRE(classified_state(ais4104::classify_slots(mixed, cases), name) == wanted);
    }
}

}

TEST_CASE("every rigid-motion slot this project composes binds an implementation of its own", "[rigid_motion][evaluation]")
{
    const praxis::rigid_motion::capabilities composed  = ais4104::rigid_motion::motions();
    const praxis::rigid_motion::capabilities reference = praxis::rigid_motion::baseline();

    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(composed, reference);
    require_named_table(compared);

    const std::array<bool, frame_slots> frames = frame_bindings_apart(composed.frame, reference.frame);
    const std::array<bool, screw_slots> screws = screw_bindings_apart(composed.screw, reference.screw);

    std::size_t index = 0;
    for(const praxis::evaluation::evaluation_view &view : compared)
    {
        for(const praxis::evaluation::slot_evaluation &slot : view.slots())
        {
            DYNAMIC_SECTION(slot.name)
            {
                REQUIRE((index < frame_slots ? frames[index] : screws[index - frame_slots]));
            }
            ++index;
        }
    }
}

TEST_CASE("classification: correct binding", "[rigid_motion][classification]")
{
    const praxis::rigid_motion::capabilities mixed = partly_written();

    require_state(mixed, "frame.rotate_x", ais4104::slot_state::written_and_correct);
}

TEST_CASE("classification: transposed binding", "[rigid_motion][classification]")
{
    const praxis::rigid_motion::capabilities mixed = partly_written();

    require_state(mixed, "frame.rotate_x", ais4104::slot_state::written_and_correct);
    require_state(mixed, "frame.rotate_z", ais4104::slot_state::written_and_disagrees);
}

TEST_CASE("classification: unwritten plain-value slot", "[rigid_motion][classification]")
{
    const praxis::rigid_motion::capabilities mixed = partly_written();

    require_state(mixed, "screw.matrix_exponential_screw", ais4104::slot_state::unwritten);
}

TEST_CASE("classification: unwritten refusal-channel slot", "[rigid_motion][classification]")
{
    const praxis::rigid_motion::capabilities mixed = partly_written();

    require_state(mixed, "screw.matrix_logarithm_se3", ais4104::slot_state::unwritten);
}

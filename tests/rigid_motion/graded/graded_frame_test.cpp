#include "case_rows.h"
#include "frame_cases.h"
#include "graded_check.h"
#include "graded_tasks.h"
#include "slot_runners.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/axis_order.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <format>
#include <cstddef>
#include <cstdint>

namespace ais4104 {
namespace {

using motions   = praxis::rigid_motion::capabilities;
using allowance = praxis::evaluation::tolerance_pair;

constexpr auto geodesic = praxis::evaluation::geodesic_residual;
constexpr auto pose     = praxis::evaluation::pose_residual;

praxis::axis_order order_of(std::uint8_t index)
{
    return static_cast<praxis::axis_order>(index);
}

// A rotation triple names an element without being one, and away from the singular configurations
// two triples name the same element, so the element each names is what is compared. The platform's
// own comparator recomposes through a function file-local to it; this is its public composition,
// which the answer passes through before it is compared against the frozen rotation the triple was
// extracted from, so it carries no answer of its own.
praxis::rotation recomposed(const Eigen::Vector3d &radians, praxis::axis_order order)
{
    static const motions reference = praxis::rigid_motion::baseline();

    return reference.frame.rotation_matrix_from_euler(radians, order);
}

praxis::evaluation::residual up_to_branch(const praxis::rotation &held, const praxis::rotation &against)
{
    const praxis::evaluation::residual between = praxis::evaluation::geodesic_residual(held, against);

    return praxis::evaluation::residual{praxis::evaluation::residual_kind::log_up_to_branch, between.magnitude, 0.0};
}

// The rows live in the frozen table; nothing here asks the platform for an answer. The array's
// length is the count of frame tasks the shared table names, so a missing row will not build.
constexpr std::array<graded_rows, tasks_named("frame.")> frame_rows{{
        {"1j",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, euler_from_rotation_matrix_cases,
                                 [&](const auto &r)
                                 {
                                     const praxis::axis_order order = order_of(r.axis_order);

                                     return graded_against(recomposed(c.frame.euler_from_rotation_matrix(matrix3_of(r.matrix), order), order), matrix3_of(r.expected), up_to_branch, a);
                                 });
         }},
        {"1d", [](const motions &c, const graded_task &e, const allowance &a)
         { return require_rows(e, a, rotate_x_cases, [&](const auto &r) { return graded_against(c.frame.rotate_x(r.radians), matrix3_of(r.expected), geodesic, a); }); }},
        {"1e", [](const motions &c, const graded_task &e, const allowance &a)
         { return require_rows(e, a, rotate_y_cases, [&](const auto &r) { return graded_against(c.frame.rotate_y(r.radians), matrix3_of(r.expected), geodesic, a); }); }},
        {"1f", [](const motions &c, const graded_task &e, const allowance &a)
         { return require_rows(e, a, rotate_z_cases, [&](const auto &r) { return graded_against(c.frame.rotate_z(r.radians), matrix3_of(r.expected), geodesic, a); }); }},
        {"1g",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, rotation_matrix_from_frame_axes_cases,
                                 [&](const auto &r)
                                 {
                                     const praxis::rotation axes = matrix3_of(r.matrix);

                                     return graded_against(c.frame.rotation_matrix_from_frame_axes(axes.col(0), axes.col(1), axes.col(2)), matrix3_of(r.expected), geodesic, a);
                                 });
         }},
        {"1i",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, rotation_matrix_from_euler_cases, [&](const auto &r)
                                 { return graded_against(c.frame.rotation_matrix_from_euler(vector_of(r.radians), order_of(r.axis_order)), matrix3_of(r.expected), geodesic, a); });
         }},
        {"1h",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, rotation_matrix_from_axis_angle_cases, [&](const auto &r)
                                 { return graded_against(c.frame.rotation_matrix_from_axis_angle(vector_of(r.direction), r.radians), matrix3_of(r.expected), geodesic, a); });
         }},
        {"2b",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, rotation_matrix_from_transform_cases,
                                 [&](const auto &r) { return graded_against(c.frame.rotation_matrix_from_transform(matrix4_of(r.transform)), matrix3_of(r.expected), geodesic, a); });
         }},
        {"2d",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, transformation_matrix_from_position_cases, [&](const auto &r)
                                 { return graded_against(c.frame.transformation_matrix_from_position(vector_of(r.position_metres)), matrix4_of(r.expected), pose, a); });
         }},
        {"2e",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, transformation_matrix_from_rotation_cases,
                                 [&](const auto &r) { return graded_against(c.frame.transformation_matrix_from_rotation(matrix3_of(r.matrix)), matrix4_of(r.expected), pose, a); });
         }},
        {"2c",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, transformation_matrix_from_rotation_position_cases,
                                 [&](const auto &r)
                                 {
                                     return graded_against(c.frame.transformation_matrix_from_rotation_position(matrix3_of(r.matrix), vector_of(r.position_metres)),
                                                           matrix4_of(r.expected), pose, a);
                                 });
         }},
        {"2f", [](const motions &c, const graded_task &e, const allowance &a)
         { return require_rows(e, a, inverse_cases, [&](const auto &r) { return graded_against(c.frame.inverse(matrix4_of(r.transform)), matrix4_of(r.expected), pose, a); }); }},
}};

static_assert(tasks_cover(frame_rows, "frame."));

}
}

TEST_CASE("every graded frame task answers what the frozen table expects", "[rigid_motion][graded]")
{
    const ais4104::motions composed = ais4104::rigid_motion::motions();

    for(const ais4104::graded_rows &graded : ais4104::frame_rows)
    {
        const ais4104::graded_task &entry = *ais4104::graded_task_of(graded.task);

        DYNAMIC_SECTION(std::format("task {} - {}", entry.task, entry.slot))
        {
            const praxis::evaluation::slot_evaluation *const slot = ais4104::graded_slot(entry.slot);
            REQUIRE(slot != nullptr);

            graded.run(composed, entry, slot->allowed);
        }
    }
}

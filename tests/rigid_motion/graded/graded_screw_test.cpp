#include "case_rows.h"
#include "screw_cases.h"
#include "graded_check.h"
#include "graded_tasks.h"
#include "slot_runners.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <format>
#include <cstddef>

namespace ais4104 {
namespace {

using motions   = praxis::rigid_motion::capabilities;
using allowance = praxis::evaluation::tolerance_pair;

constexpr auto element_wise    = praxis::evaluation::element_wise_residual;
constexpr auto geodesic        = praxis::evaluation::geodesic_residual;
constexpr auto pose            = praxis::evaluation::pose_residual;
constexpr auto axis_up_to_sign = praxis::evaluation::axis_up_to_sign_residual;

// The rows live in the frozen table; nothing here asks the platform for an answer. The array's
// length is the count of screw tasks the shared table names, so a missing row will not build.
constexpr std::array<graded_rows, tasks_named("screw.")> screw_rows{{
        {"1b",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, skew_symmetric_cases,
                                 [&](const auto &r) { return graded_against(c.screw.skew_symmetric(vector_of(r.vector)), matrix3_of(r.expected), element_wise, a); });
         }},
        {"1c",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, from_skew_symmetric_cases,
                                 [&](const auto &r) { return graded_against(c.screw.from_skew_symmetric(matrix3_of(r.matrix)), vector_of(r.expected), element_wise, a); });
         }},
        {"3f",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, adjoint_matrix_from_rotation_position_cases, [&](const auto &r)
                                 { return graded_against(c.screw.adjoint_matrix_from_rotation_position(matrix3_of(r.rotation), vector_of(r.position_metres)), adjoint_of(r.expected), element_wise, a); });
         }},
        {"3g",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, adjoint_matrix_from_transform_cases,
                                 [&](const auto &r) { return graded_against(c.screw.adjoint_matrix_from_transform(matrix4_of(r.transform)), adjoint_of(r.expected), element_wise, a); });
         }},
        {"3h",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, adjoint_map_cases, [&](const auto &r)
                                 { return graded_against(c.screw.adjoint_map(vector_of(r.twist), matrix4_of(r.transform)), vector_of(r.expected), element_wise, a); });
         }},
        {"3d",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, twist_from_angular_linear_cases, [&](const auto &r)
                                 { return graded_against(c.screw.twist_from_angular_linear(vector_of(r.angular), vector_of(r.linear)), vector_of(r.expected), element_wise, a); });
         }},
        {"3e",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, twist_from_screw_cases,
                                 [&](const auto &r)
                                 {
                                     return graded_against(c.screw.twist_from_screw(vector_of(r.point_metres), vector_of(r.direction), r.pitch, r.angular_velocity),
                                                           vector_of(r.expected), element_wise, a);
                                 });
         }},
        {"3i",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(
                     e, a, twist_matrix_from_angular_linear_cases, [&](const auto &r)
                     { return graded_against(c.screw.twist_matrix_from_angular_linear(vector_of(r.angular), vector_of(r.linear)), matrix4_of(r.expected), element_wise, a); });
         }},
        {"3j",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, twist_matrix_from_twist_cases,
                                 [&](const auto &r) { return graded_against(c.screw.twist_matrix_from_twist(vector_of(r.twist)), matrix4_of(r.expected), element_wise, a); });
         }},
        {"3c",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(
                     e, a, screw_axis_from_angular_linear_cases, [&](const auto &r)
                     { return graded_against(c.screw.screw_axis_from_angular_linear(vector_of(r.angular), vector_of(r.linear)), vector_of(r.expected), axis_up_to_sign, a); });
         }},
        {"3b",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, screw_axis_from_point_direction_pitch_cases, [&](const auto &r)
                                 { return graded_against(c.screw.screw_axis_from_point_direction_pitch(vector_of(r.point_metres), vector_of(r.direction), r.pitch), vector_of(r.expected_axis), axis_up_to_sign, a); });
         }},
        {"3k",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_exponential_so3_cases,
                                 [&](const auto &r) { return graded_against(c.screw.matrix_exponential_so3(vector_of(r.direction), r.radians), matrix3_of(r.expected), geodesic, a); });
         }},
        {"3l",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_exponential_se3_cases, [&](const auto &r)
                                 { return graded_against(c.screw.matrix_exponential_se3(vector_of(r.angular), vector_of(r.linear), r.radians), matrix4_of(r.expected), pose, a); });
         }},
        {"3m",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_exponential_screw_cases,
                                 [&](const auto &r) { return graded_against(c.screw.matrix_exponential_screw(vector_of(r.axis), r.radians), matrix4_of(r.expected), pose, a); });
         }},
        {"3n",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_logarithm_so3_cases, [&](const auto &r)
                                 { return graded_against(c.screw.matrix_logarithm_so3(matrix3_of(r.rotation)), rotation_log{vector_of(r.expected_axis), r.expected_radians}, rotation_log_residual, a); });
         }},
        {"3o",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_logarithm_se3_rp_cases, [&](const auto &r)
                                 { return graded_against(c.screw.matrix_logarithm_se3_rp(matrix3_of(r.rotation), vector_of(r.position_metres)), pose_log{vector_of(r.expected_axis), r.expected_radians}, pose_log_residual, a); });
         }},
        {"3p",
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(
                     e, a, matrix_logarithm_se3_cases, [&](const auto &r)
                     { return graded_against(c.screw.matrix_logarithm_se3(matrix4_of(r.transform)), pose_log{vector_of(r.expected_axis), r.expected_radians}, pose_log_residual, a); });
         }},
}};

static_assert(tasks_cover(screw_rows, "screw."));

}
}

TEST_CASE("every graded screw task answers what the frozen table expects", "[rigid_motion][graded]")
{
    const ais4104::motions composed = ais4104::rigid_motion::motions();

    for(const ais4104::graded_rows &graded : ais4104::screw_rows)
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

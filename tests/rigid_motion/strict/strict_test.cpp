#include "case_rows.h"
#include "graded_check.h"
#include "graded_tasks.h"
#include "strict_cases.h"
#include "strict_bounds.h"

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
#include <string_view>

namespace ais4104 {
namespace {

using motions   = praxis::rigid_motion::capabilities;
using allowance = praxis::evaluation::tolerance_pair;

constexpr auto element_wise    = praxis::evaluation::element_wise_residual;
constexpr auto geodesic        = praxis::evaluation::geodesic_residual;
constexpr auto pose            = praxis::evaluation::pose_residual;
constexpr auto axis_up_to_sign = praxis::evaluation::axis_up_to_sign_residual;

praxis::axis_order order_of(std::uint8_t index)
{
    return static_cast<praxis::axis_order>(index);
}

// One entry per slot the measurement grades strictly: the assignment's own identifier, the bound the
// record gives that slot, and how that slot's rows are answered and judged. A slot whose printed
// branch the drawn corpus cannot reach runs its authored row after its drawn ones, at a case
// identifier continuing past them. This suite carries value rows only; a refusal is graded nowhere
// but in the graded suite.
struct strict_rows
{
    std::string_view task;
    const strict_bound *bound;
    std::size_t (*run)(const motions &, const graded_task &, const allowance &);
};

constexpr std::array<strict_rows, strictly_graded_slots> strict_slots{{
        {"1i", &rotation_matrix_from_euler_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, rotation_matrix_from_euler_near_singular_cases, [&](const auto &r)
                                 { return graded_against(c.frame.rotation_matrix_from_euler(vector_of(r.radians), order_of(r.axis_order)), matrix3_of(r.expected), geodesic, a); });
         }},
        {"1h", &rotation_matrix_from_axis_angle_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, rotation_matrix_from_axis_angle_near_singular_cases, [&](const auto &r)
                                 { return graded_against(c.frame.rotation_matrix_from_axis_angle(vector_of(r.direction), r.radians), matrix3_of(r.expected), geodesic, a); });
         }},
        {"3f", &adjoint_matrix_from_rotation_position_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, adjoint_matrix_from_rotation_position_near_singular_cases,
                                 [&](const auto &r)
                                 {
                                     return graded_against(c.screw.adjoint_matrix_from_rotation_position(matrix3_of(r.rotation), vector_of(r.position_metres)), adjoint_of(r.expected),
                                                           element_wise, a);
                                 });
         }},
        {"3g", &adjoint_matrix_from_transform_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, adjoint_matrix_from_transform_near_singular_cases,
                                 [&](const auto &r) { return graded_against(c.screw.adjoint_matrix_from_transform(matrix4_of(r.transform)), adjoint_of(r.expected), element_wise, a); });
         }},
        {"3h", &adjoint_map_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, adjoint_map_near_singular_cases, [&](const auto &r)
                                 { return graded_against(c.screw.adjoint_map(vector_of(r.twist), matrix4_of(r.transform)), vector_of(r.expected), element_wise, a); });
         }},
        {"3c", &screw_axis_from_angular_linear_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, screw_axis_from_angular_linear_near_singular_cases, [&](const auto &r)
                                 { return graded_against(c.screw.screw_axis_from_angular_linear(vector_of(r.angular), vector_of(r.linear)), vector_of(r.expected), axis_up_to_sign, a); });
         }},
        {"3k", &matrix_exponential_so3_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             const auto judged = [&](const auto &r) { return graded_against(c.screw.matrix_exponential_so3(vector_of(r.direction), r.radians), matrix3_of(r.expected), geodesic, a); };
             const std::size_t counted = require_rows(e, a, matrix_exponential_so3_near_singular_cases, judged);

             return counted + require_rows(e, a, matrix_exponential_so3_guard_band_cases, judged, counted);
         }},
        {"3l", &matrix_exponential_se3_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_exponential_se3_near_singular_cases, [&](const auto &r)
                                 { return graded_against(c.screw.matrix_exponential_se3(vector_of(r.angular), vector_of(r.linear), r.radians), matrix4_of(r.expected), pose, a); });
         }},
        {"3m", &matrix_exponential_screw_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_exponential_screw_near_singular_cases,
                                 [&](const auto &r) { return graded_against(c.screw.matrix_exponential_screw(vector_of(r.axis), r.radians), matrix4_of(r.expected), pose, a); });
         }},
        {"3n", &matrix_logarithm_so3_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(e, a, matrix_logarithm_so3_near_singular_cases,
                                 [&](const auto &r)
                                 {
                                     return graded_against(c.screw.matrix_logarithm_so3(matrix3_of(r.rotation)), rotation_log{vector_of(r.expected_axis), r.expected_radians},
                                                           rotation_log_residual, a);
                                 });
         }},
        {"3o", &matrix_logarithm_se3_rp_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             const auto judged = [&](const auto &r)
             {
                 return graded_against(c.screw.matrix_logarithm_se3_rp(matrix3_of(r.rotation), vector_of(r.position_metres)), pose_log{vector_of(r.expected_axis), r.expected_radians},
                                       pose_log_residual, a);
             };
             const std::size_t counted = require_rows(e, a, matrix_logarithm_se3_rp_near_singular_cases, judged);

             return counted + require_rows(e, a, matrix_logarithm_se3_rp_guard_band_cases, judged, counted);
         }},
        {"3p", &matrix_logarithm_se3_bound,
         [](const motions &c, const graded_task &e, const allowance &a)
         {
             return require_rows(
                     e, a, matrix_logarithm_se3_near_singular_cases, [&](const auto &r)
                     { return graded_against(c.screw.matrix_logarithm_se3(matrix4_of(r.transform)), pose_log{vector_of(r.expected_axis), r.expected_radians}, pose_log_residual, a); });
         }},
}};

static_assert(tasks_are_distinct(strict_slots));

}
}

// The leading period hides every case here, so the suite registers no test with the runner and a red
// row in it cannot turn the run a student is graded by red. Selecting the tag explicitly runs it.
TEST_CASE("every strict task answers within the measured bound", "[.strict][rigid_motion]")
{
    const ais4104::motions composed = ais4104::rigid_motion::motions();

    for(const ais4104::strict_rows &graded : ais4104::strict_slots)
    {
        const ais4104::graded_task &entry = *ais4104::graded_task_of(graded.task);

        DYNAMIC_SECTION(std::format("task {} - {}", entry.task, entry.slot))
        {
            const praxis::evaluation::slot_evaluation *const slot = ais4104::graded_slot(entry.slot);
            REQUIRE(slot != nullptr);

            ais4104::standing_matches(*graded.bound, slot->allowed);
            graded.run(composed, entry, ais4104::allowed_of(*graded.bound));
        }
    }
}

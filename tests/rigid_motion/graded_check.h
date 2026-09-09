#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_GRADED_CHECK_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_GRADED_CHECK_H

#include "graded_tasks.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/types.h>
#include <praxis/rigid_motion/evaluation.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>
#include <praxis/evaluation/slot_evaluation.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <Eigen/Core>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <format>
#include <string>
#include <cstddef>
#include <utility>

namespace ais4104 {

using rotation_log = std::pair<Eigen::Vector3d, double>;
using pose_log     = std::pair<praxis::screw_axis, double>;

template<typename T>
using answer = praxis::expected<T, praxis::refusal>;

// A logarithm names a group element without being one, so the pair is unpacked and the elements it
// names are compared, exactly as the platform's own comparator does.
inline praxis::evaluation::residual rotation_log_residual(const rotation_log &held, const rotation_log &against)
{
    return praxis::evaluation::log_up_to_branch_rotation_residual(held.first, held.second, against.first, against.second);
}

inline praxis::evaluation::residual pose_log_residual(const pose_log &held, const pose_log &against)
{
    return praxis::evaluation::log_up_to_branch_pose_residual(held.first, held.second, against.first, against.second);
}

inline const std::array<praxis::evaluation::evaluation_view, 2> &graded_views()
{
    static const praxis::rigid_motion::capabilities composed  = ais4104::rigid_motion::motions();
    static const praxis::rigid_motion::capabilities reference = praxis::rigid_motion::baseline();

    static const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(composed, reference);

    return compared;
}

// The platform's own view of a slot supplies its name, its residual kind and the tolerance pair
// every graded row is judged at. No answer is ever taken from it: the expected value comes from the
// frozen table, and the line it is held to is the one the platform publishes for that kind.
inline const praxis::evaluation::slot_evaluation *graded_slot(std::string_view name)
{
    for(const praxis::evaluation::evaluation_view &view : graded_views())
        for(const praxis::evaluation::slot_evaluation &slot : view.slots())
            if(slot.name == name)
                return &slot;

    return nullptr;
}

// The suite's own fold rather than the platform's verdict, which scores a non-finite residual as
// agreement because a NaN compares false against every bound.
inline bool case_passed(const praxis::evaluation::case_result &answer)
{
    if(!std::isfinite(answer.difference.magnitude) || !std::isfinite(answer.difference.linear_error_metres))
        return false;

    return answer.verdict == praxis::evaluation::agreement::agreed || answer.verdict == praxis::evaluation::agreement::both_refused;
}

inline std::string graded_line(const graded_task &entry, std::size_t index, const praxis::evaluation::residual &seen, const praxis::evaluation::tolerance_pair &allowed)
{
    return std::format("task {} - {} case {} residual {:.6e} / {:.6e} allowed {:.6e} / {:.6e}", entry.task, entry.slot, index, seen.magnitude, seen.linear_error_metres,
                       allowed.magnitude, allowed.linear_metres);
}

inline void require_graded_case(const praxis::evaluation::case_result &judged, const graded_task &entry, std::size_t index, const praxis::evaluation::tolerance_pair &allowed)
{
    INFO(graded_line(entry, index, judged.difference, allowed));
    REQUIRE(case_passed(judged));
}

// A slot that cannot refuse is judged by the platform's verdict over the residual alone; one that can
// goes through the platform's own refusal-aware fold, so a refusal is never turned into a number.
template<typename T, typename Compare>
praxis::evaluation::case_result graded_against(const T &held, const T &expected, Compare compared, const praxis::evaluation::tolerance_pair &allowed)
{
    const praxis::evaluation::residual difference = compared(held, expected);

    return praxis::evaluation::case_result{praxis::evaluation::verdict_of(difference, allowed), difference};
}

template<typename T, typename Compare>
praxis::evaluation::case_result graded_against(const answer<T> &held, const T &expected, Compare compared, const praxis::evaluation::tolerance_pair &allowed)
{
    return praxis::evaluation::agreed_or_refused(held, answer<T>{expected}, compared, allowed);
}

template<typename T, typename Compare>
praxis::evaluation::case_result refused_against(const answer<T> &held, praxis::refusal expected, Compare compared, const praxis::evaluation::tolerance_pair &allowed)
{
    return praxis::evaluation::agreed_or_refused(held, answer<T>{praxis::unexpected(expected)}, compared, allowed);
}

// A refusal row agrees numerically with nothing, so the verdict carries the whole result: the two
// sides must have declined the same input for the same reason.
inline void require_refused_case(const praxis::evaluation::case_result &judged, const graded_task &entry, std::size_t index, const praxis::evaluation::tolerance_pair &allowed)
{
    require_graded_case(judged, entry, index, allowed);
    REQUIRE(judged.verdict == praxis::evaluation::agreement::both_refused);
}

// Every row of a slot in turn, each judged at the pair the platform publishes for that slot's kind.
// A refusal row's case identifier continues past the answering rows, so a failure names it uniquely.
template<typename Rows, typename Judge>
std::size_t require_rows(const graded_task &entry, const praxis::evaluation::tolerance_pair &allowed, const Rows &rows, Judge judged, std::size_t offset = 0)
{
    for(std::size_t index = 0; index < rows.size(); ++index)
        require_graded_case(judged(rows[index]), entry, offset + index, allowed);

    return rows.size();
}

template<typename Rows, typename Judge>
std::size_t require_refusals(const graded_task &entry, const praxis::evaluation::tolerance_pair &allowed, const Rows &rows, Judge judged, std::size_t offset = 0)
{
    for(std::size_t index = 0; index < rows.size(); ++index)
        require_refused_case(judged(rows[index]), entry, offset + index, allowed);

    return rows.size();
}

}

#endif

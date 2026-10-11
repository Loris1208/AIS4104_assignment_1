#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_2_MANIPULATOR_ROWS_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_2_MANIPULATOR_ROWS_H

#include "manipulator_cases.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/slot_evaluation.h>

#include <praxis/manipulator/types.h>
#include <praxis/manipulator/capabilities.h>

#include <praxis/rigid_motion/types.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace ais4104 {

template<typename T>
using answer = praxis::expected<T, praxis::refusal>;

enum class bound_standing : std::uint8_t
{
    tighter,
    at_the_platform,
    above_the_platform,
};

// Radians and meters for a pose; the greatest entry difference as the magnitude otherwise, the linear figure unread.
struct pinned_bound
{
    double magnitude;
    double linear_metres;
    bound_standing magnitude_standing;
    bound_standing linear_standing;
};

constexpr pinned_bound forward_kinematics_bound{1.0e-13, 1.0e-11, bound_standing::at_the_platform, bound_standing::at_the_platform};
constexpr pinned_bound body_screws_bound{1.0e-13, 1.0e-13, bound_standing::at_the_platform, bound_standing::at_the_platform};
constexpr pinned_bound body_forward_kinematics_bound{1.0e-13, 1.0e-11, bound_standing::at_the_platform, bound_standing::at_the_platform};
constexpr pinned_bound space_jacobian_bound{1.0e-11, 1.0e-11, bound_standing::at_the_platform, bound_standing::at_the_platform};
constexpr pinned_bound body_jacobian_bound{1.0e-11, 1.0e-11, bound_standing::at_the_platform, bound_standing::at_the_platform};

constexpr praxis::evaluation::tolerance_pair allowed_of(const pinned_bound &bound)
{
    return praxis::evaluation::tolerance_pair{bound.magnitude, bound.linear_metres};
}

constexpr bound_standing standing_of(double bound, double published)
{
    if(bound < published)
        return bound_standing::tighter;
    if(published < bound)
        return bound_standing::above_the_platform;

    return bound_standing::at_the_platform;
}

struct case_reading
{
    praxis::evaluation::case_result judged;
    std::string clause;
};

struct manipulator_row
{
    std::string_view task;
    std::string_view slot;
    std::string_view published;
    pinned_bound bound;
    case_reading (*read)(const frozen_chain &chain, const pinned_bound &bound);
};

template<typename T>
std::optional<praxis::refusal> refusal_of(const answer<T> &given)
{
    if(given.has_value())
        return std::nullopt;

    return given.error();
}

const praxis::rigid_motion::capabilities &reference_motions();

const praxis::manipulator::capabilities &reference_manipulator();

std::string refusal_clause(std::optional<praxis::refusal> held, std::optional<praxis::refusal> against);

std::string residual_clause(const praxis::evaluation::residual &seen, const praxis::evaluation::tolerance_pair &allowed);

case_reading pose_reading(const answer<Eigen::Matrix4d> &held, const answer<Eigen::Matrix4d> &against, const pinned_bound &bound);

case_reading jacobian_reading(const answer<praxis::manipulator::jacobian> &held, const answer<praxis::manipulator::jacobian> &against, const pinned_bound &bound);

case_reading screw_axes_reading(const answer<std::vector<praxis::screw_axis>> &held, const answer<std::vector<praxis::screw_axis>> &against, const pinned_bound &bound);

bool passed(const case_reading &read);

std::string row_line(const manipulator_row &row, std::size_t index, const case_reading &read);

void check_row(const manipulator_row &row);

}

#endif

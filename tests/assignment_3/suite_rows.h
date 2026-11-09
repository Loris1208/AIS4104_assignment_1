#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_3_SUITE_ROWS_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_3_SUITE_ROWS_H

#include <praxis/evaluation/residual.h>

#include <praxis/manipulator/capabilities.h>

#include <praxis/trajectory/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/optimization/capabilities.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <string>
#include <cstddef>
#include <cstdint>
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
    bool passed;
    std::string clause;
};

struct graded_row
{
    std::string_view task;
    std::string_view slot;
    std::string_view published;
    pinned_bound bound;
    std::size_t (*run)(const graded_row &row);
};

const praxis::rigid_motion::capabilities &reference_motions();

const praxis::manipulator::capabilities &reference_manipulator();

const praxis::trajectory::capabilities &reference_trajectory();

const praxis::optimization::capabilities &reference_solving();

const praxis::optimization::capabilities &student_solving();

std::string_view refusal_name(praxis::refusal reason);

std::string refused_clause(praxis::refusal reason);

std::string residual_clause(const praxis::evaluation::residual &seen, const praxis::evaluation::tolerance_pair &allowed);

std::string row_line(std::string_view task, std::string_view slot, std::string_view subject, std::size_t index, std::string_view clause);

void check_row(const graded_row &row);

void check_row(const graded_row &row, std::string_view extension);

}

#endif

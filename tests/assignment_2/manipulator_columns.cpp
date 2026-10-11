#include "manipulator_rows.h"

#include <praxis/evaluation/comparators.h>

#include <Eigen/Core>

#include <format>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace ais4104 {
namespace {

case_reading shape_reading(Eigen::Index held_columns, Eigen::Index against_columns, std::string_view noun)
{
    const praxis::evaluation::residual apart{praxis::evaluation::residual_kind::element_wise, std::numeric_limits<double>::infinity(), 0.0};

    return case_reading{praxis::evaluation::case_result{praxis::evaluation::agreement::differed, apart},
                        std::format(" answers {} {}, the chain has {} joints", held_columns, noun, against_columns)};
}

// Numbered from 1, as the book numbers joints.
std::vector<Eigen::Index> differing_columns(const praxis::manipulator::jacobian &held, const praxis::manipulator::jacobian &against, double magnitude)
{
    std::vector<Eigen::Index> numbers;
    for(Eigen::Index column = 0; column < held.cols(); ++column)
        if(!((held.col(column) - against.col(column)).cwiseAbs().maxCoeff<Eigen::PropagateNaN>() <= magnitude))
            numbers.push_back(column + 1);

    return numbers;
}

std::string columns_clause(const std::vector<Eigen::Index> &numbers)
{
    if(numbers.empty())
        return std::string();
    if(numbers.size() == 1)
        return std::format("; column {} differs", numbers.front());

    std::string listed = std::format("{}", numbers.front());
    for(std::size_t index = 1; index < numbers.size(); ++index)
        listed += std::format(", {}", numbers[index]);

    return std::format("; columns {} differ", listed);
}

std::string element_clause(const praxis::evaluation::residual &seen, const praxis::evaluation::tolerance_pair &allowed)
{
    return std::format(" largest entry difference {:.6e} allowed {:.6e}", seen.magnitude, allowed.magnitude);
}

answer<praxis::manipulator::jacobian> as_columns(const answer<std::vector<praxis::screw_axis>> &axes)
{
    if(!axes)
        return praxis::unexpected(axes.error());

    praxis::manipulator::jacobian columns(6, static_cast<Eigen::Index>(axes->size()));
    for(std::size_t index = 0; index < axes->size(); ++index)
        columns.col(static_cast<Eigen::Index>(index)) = (*axes)[index];

    return columns;
}

case_reading element_wise_reading(const answer<praxis::manipulator::jacobian> &held, const answer<praxis::manipulator::jacobian> &against, const pinned_bound &bound,
                                  std::string_view noun)
{
    if(held && against && held->cols() != against->cols())
        return shape_reading(held->cols(), against->cols(), noun);

    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const praxis::evaluation::case_result judged     = praxis::evaluation::agreed_or_refused(held, against, praxis::evaluation::element_wise_residual, allowed);

    if(!held || !against)
        return case_reading{judged, refusal_clause(refusal_of(held), refusal_of(against))};

    return case_reading{judged, element_clause(judged.difference, allowed)};
}

}

case_reading jacobian_reading(const answer<praxis::manipulator::jacobian> &held, const answer<praxis::manipulator::jacobian> &against, const pinned_bound &bound)
{
    case_reading read = element_wise_reading(held, against, bound, "columns");
    if(held && against && held->cols() == against->cols() && read.judged.verdict == praxis::evaluation::agreement::differed)
        read.clause += columns_clause(differing_columns(*held, *against, bound.magnitude));

    return read;
}

case_reading screw_axes_reading(const answer<std::vector<praxis::screw_axis>> &held, const answer<std::vector<praxis::screw_axis>> &against, const pinned_bound &bound)
{
    return element_wise_reading(as_columns(held), as_columns(against), bound, "screw axes");
}

}

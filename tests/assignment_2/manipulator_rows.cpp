#include "manipulator_rows.h"
#include "manipulator_cases.h"

#include <praxis/manipulator/evaluation.h>

#include <praxis/evaluation/comparators.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <format>

namespace ais4104 {
namespace {

std::optional<praxis::evaluation::tolerance_pair> published_allowance(std::string_view name)
{
    static const std::array<praxis::evaluation::evaluation_view, 7> views = praxis::manipulator::evaluation_views(reference_manipulator(), reference_manipulator());

    for(const praxis::evaluation::evaluation_view &view : views)
        for(const praxis::evaluation::slot_evaluation &slot : view.slots())
            if(slot.name == name)
                return slot.allowed;

    return std::nullopt;
}

std::string standing_line(const manipulator_row &row, std::string_view half, double bound, double published)
{
    return std::format("{}: the {} bound {:.17e} against the published {:.17e}", row.published, half, bound, published);
}

void standing_matches(const manipulator_row &row, const praxis::evaluation::tolerance_pair &published)
{
    {
        INFO(standing_line(row, "magnitude", row.bound.magnitude, published.magnitude));
        REQUIRE(standing_of(row.bound.magnitude, published.magnitude) == row.bound.magnitude_standing);
    }
    {
        INFO(standing_line(row, "linear", row.bound.linear_metres, published.linear_metres));
        REQUIRE(standing_of(row.bound.linear_metres, published.linear_metres) == row.bound.linear_standing);
    }
}

std::string_view refusal_name(praxis::refusal reason)
{
    switch(reason)
    {
        case praxis::refusal::unsupported_input:
            return "unsupported_input";
        case praxis::refusal::degenerate:
            return "degenerate";
        case praxis::refusal::no_solution:
            return "no_solution";
        case praxis::refusal::not_implemented:
            break;
    }

    return "not_implemented";
}

bool reference_answered(const case_reading &read)
{
    return read.judged.verdict != praxis::evaluation::agreement::both_refused && read.judged.verdict != praxis::evaluation::agreement::refused_differently;
}

}

const praxis::rigid_motion::capabilities &reference_motions()
{
    static const praxis::rigid_motion::capabilities motions = praxis::rigid_motion::baseline();

    return motions;
}

const praxis::manipulator::capabilities &reference_manipulator()
{
    static const praxis::manipulator::capabilities manipulator = praxis::manipulator::baseline();

    return manipulator;
}

std::string refusal_clause(std::optional<praxis::refusal> held, std::optional<praxis::refusal> against)
{
    if(held && against && *held == *against)
        return std::format(" refused ({}) as the reference did", refusal_name(*held));
    if(held && against)
        return std::format(" answered nothing ({}) where the reference refused ({})", refusal_name(*held), refusal_name(*against));
    if(held)
        return std::format(" answered nothing ({})", refusal_name(*held));
    if(against)
        return std::format(" answered where the reference refused ({})", refusal_name(*against));

    return std::string();
}

std::string residual_clause(const praxis::evaluation::residual &seen, const praxis::evaluation::tolerance_pair &allowed)
{
    return std::format(" residual {:.6e} / {:.6e} allowed {:.6e} / {:.6e}", seen.magnitude, seen.linear_error_metres, allowed.magnitude, allowed.linear_metres);
}

case_reading pose_reading(const answer<Eigen::Matrix4d> &held, const answer<Eigen::Matrix4d> &against, const pinned_bound &bound)
{
    const praxis::evaluation::tolerance_pair allowed = allowed_of(bound);
    const praxis::evaluation::case_result judged     = praxis::evaluation::agreed_or_refused(held, against, praxis::evaluation::pose_residual, allowed);

    if(!held || !against)
        return case_reading{judged, refusal_clause(refusal_of(held), refusal_of(against))};

    return case_reading{judged, residual_clause(judged.difference, allowed)};
}

bool passed(const case_reading &read)
{
    return read.judged.verdict == praxis::evaluation::agreement::agreed || read.judged.verdict == praxis::evaluation::agreement::both_refused;
}

std::string row_line(const manipulator_row &row, std::size_t index, const case_reading &read)
{
    return std::format("task {} - {} case {}{}", row.task, row.slot, index, read.clause);
}

void check_row(const manipulator_row &row)
{
    const std::optional<praxis::evaluation::tolerance_pair> published = published_allowance(row.published);
    {
        INFO(std::format("praxis publishes no figure for {}", row.published));
        REQUIRE(published.has_value());
    }
    standing_matches(row, *published);

    for(std::size_t index = 0; index < frozen_chains.size(); ++index)
    {
        const case_reading read = row.read(frozen_chains[index], row.bound);
        INFO(row_line(row, index, read));
        CHECK(reference_answered(read));
        CHECK(passed(read));
    }
}

}

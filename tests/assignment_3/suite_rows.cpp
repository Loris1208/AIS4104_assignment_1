#include "suite_rows.h"

#include "optimization/capabilities.h"

#include <praxis/evaluation/slot_evaluation.h>

#include <praxis/manipulator/evaluation.h>

#include <praxis/trajectory/evaluation.h>

#include <praxis/optimization/evaluation.h>

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <format>
#include <vector>
#include <initializer_list>

namespace ais4104 {
namespace {

struct published_figure
{
    std::string_view extension;
    praxis::evaluation::tolerance_pair allowed;
};

using view_span = std::span<const praxis::evaluation::evaluation_view>;

std::vector<published_figure> published_figures(std::string_view name)
{
    static const std::array<praxis::evaluation::evaluation_view, 7> manipulator_views = praxis::manipulator::evaluation_views(reference_manipulator(), reference_manipulator());
    static const std::array<praxis::evaluation::evaluation_view, 4> trajectory_views  = praxis::trajectory::evaluation_views(reference_trajectory(), reference_trajectory());
    static const std::array<praxis::evaluation::evaluation_view, 1> route_views       = praxis::manipulator::solver_evaluation_views(reference_solving(), reference_solving());
    static const std::array<praxis::evaluation::evaluation_view, 2> solver_views      = praxis::optimization::evaluation_views(reference_solving(), reference_solving());

    std::vector<published_figure> found;
    for(const view_span views : {view_span(manipulator_views), view_span(trajectory_views), view_span(route_views), view_span(solver_views)})
        for(const praxis::evaluation::evaluation_view &view : views)
            for(const praxis::evaluation::slot_evaluation &slot : view.slots())
                if(slot.name == name)
                    found.push_back(published_figure{view.extension(), slot.allowed});

    return found;
}

std::string publication_line(std::string_view name, const std::vector<published_figure> &found)
{
    if(found.empty())
        return std::format("praxis publishes no figure for {}", name);

    std::string under;
    for(const published_figure &each : found)
        under += (under.empty() ? "" : ", ") + std::string(each.extension);

    return std::format("praxis publishes {} figures for {}, under {}", found.size(), name, under);
}

std::string standing_line(const graded_row &row, std::string_view half, double bound, double published)
{
    return std::format("{}: the {} bound {:.17e} against the published {:.17e}", row.published, half, bound, published);
}

void standing_matches(const graded_row &row, const praxis::evaluation::tolerance_pair &published)
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

void check_published(const graded_row &row, const std::vector<published_figure> &found)
{
    {
        INFO(publication_line(row.published, found));
        REQUIRE(found.size() == 1);
    }
    standing_matches(row, found.front().allowed);

    REQUIRE(row.run(row) > 0);
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

const praxis::trajectory::capabilities &reference_trajectory()
{
    static const praxis::trajectory::capabilities trajectory = praxis::trajectory::baseline();

    return trajectory;
}

const praxis::optimization::capabilities &reference_solving()
{
    static const praxis::optimization::capabilities solving = praxis::optimization::baseline();

    return solving;
}

const praxis::optimization::capabilities &student_solving()
{
    static const praxis::optimization::capabilities solving = []
    {
        praxis::optimization::capabilities composed = praxis::optimization::baseline();
        composed.solver                             = optimization::solvers();
        composed.minimizer                          = optimization::minimizers();

        return composed;
    }();

    return solving;
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

std::string refused_clause(praxis::refusal reason)
{
    return std::format(" refused ({})", refusal_name(reason));
}

std::string residual_clause(const praxis::evaluation::residual &seen, const praxis::evaluation::tolerance_pair &allowed)
{
    return std::format(" residual {:.6e} / {:.6e} allowed {:.6e} / {:.6e}", seen.magnitude, seen.linear_error_metres, allowed.magnitude, allowed.linear_metres);
}

std::string row_line(std::string_view task, std::string_view slot, std::string_view subject, std::size_t index, std::string_view clause)
{
    if(subject.empty())
        return std::format("task {} - {} case {}{}", task, slot, index, clause);

    return std::format("task {} - {} {} case {}{}", task, slot, subject, index, clause);
}

void check_row(const graded_row &row)
{
    check_published(row, published_figures(row.published));
}

void check_row(const graded_row &row, std::string_view extension)
{
    std::vector<published_figure> found = published_figures(row.published);
    std::erase_if(found, [extension](const published_figure &each) { return each.extension != extension; });

    check_published(row, found);
}

}

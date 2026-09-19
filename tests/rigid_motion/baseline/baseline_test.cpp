#include "graded_tasks.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/evaluation.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/report.h>
#include <praxis/evaluation/generation.h>
#include <praxis/evaluation/slot_evaluation.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace {

constexpr std::size_t graded_cases = 1000;

struct spread_result
{
    bool finite;
    std::size_t exercised;
    std::size_t worst_case_index;
    praxis::evaluation::residual worst;
    praxis::evaluation::outcome_counts outcomes;
};

struct graded_row
{
    spread_result bulk;
    spread_result near_singular;
    praxis::evaluation::tolerance_pair allowed;
    std::size_t bound_measured_to_cases;
};

std::size_t *counter_for(praxis::evaluation::outcome_counts &counts, praxis::evaluation::agreement verdict)
{
    switch(verdict)
    {
        case praxis::evaluation::agreement::agreed:
            return &counts.agreed;
        case praxis::evaluation::agreement::differed:
            return &counts.differed;
        case praxis::evaluation::agreement::one_refused:
            return &counts.one_refused;
        case praxis::evaluation::agreement::both_refused:
            return &counts.both_refused;
        case praxis::evaluation::agreement::refused_differently:
            return &counts.refused_differently;
        case praxis::evaluation::agreement::unusable:
            return &counts.unusable;
        case praxis::evaluation::agreement::not_exercised:
        case praxis::evaluation::agreement::beyond_measurement:
            break;
    }

    return nullptr;
}

void recorded(spread_result &tallied, const praxis::evaluation::case_result &answer, std::size_t index)
{
    if(std::size_t *const reached = counter_for(tallied.outcomes, answer.verdict); reached != nullptr)
        ++*reached;

    if(answer.verdict != praxis::evaluation::agreement::not_exercised)
        ++tallied.exercised;

    tallied.finite = tallied.finite && std::isfinite(answer.difference.magnitude) && std::isfinite(answer.difference.linear_error_metres);

    if(answer.difference.magnitude > tallied.worst.magnitude)
    {
        tallied.worst            = answer.difference;
        tallied.worst_case_index = index;
    }
}

spread_result measured(const praxis::evaluation::evaluation_view &view, const praxis::evaluation::slot_evaluation &slot, praxis::evaluation::spread drawn_from, std::size_t cases)
{
    const praxis::evaluation::tolerance_pair allowed = praxis::evaluation::tolerance_of(slot.kind);
    spread_result tallied{true, 0, 0, praxis::evaluation::residual{slot.kind, 0.0, 0.0}, praxis::evaluation::outcome_counts{}};

    for(std::size_t index = 0; index < cases; ++index)
    {
        praxis::evaluation::case_source drawn = praxis::evaluation::case_source::at_case(praxis::evaluation::default_seed, slot.name, drawn_from, index);
        recorded(tallied, slot.compare(view.first(), view.second(), drawn, allowed), index);
    }

    return tallied;
}

bool exercised_throughout(const spread_result &tallied, std::size_t cases)
{
    return cases > 0 && tallied.exercised == cases;
}

bool agreed_throughout(const spread_result &tallied, std::size_t cases)
{
    return tallied.outcomes.agreed + tallied.outcomes.both_refused == cases;
}

graded_row row_for(const praxis::evaluation::evaluation_view &view, const praxis::evaluation::slot_evaluation &slot)
{
    return graded_row{measured(view, slot, praxis::evaluation::spread::bulk, graded_cases), measured(view, slot, praxis::evaluation::spread::near_singular, graded_cases),
                      praxis::evaluation::tolerance_of(slot.kind), slot.bound_measured_to_cases};
}

std::vector<graded_row> graded_rows()
{
    const praxis::rigid_motion::capabilities composed  = ais4104::rigid_motion::motions();
    const praxis::rigid_motion::capabilities reference = praxis::rigid_motion::baseline();

    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(composed, reference);

    std::vector<graded_row> rows;
    rows.reserve(ais4104::graded_tasks.size());
    for(const praxis::evaluation::evaluation_view &view : compared)
        for(const praxis::evaluation::slot_evaluation &slot : view.slots())
            rows.push_back(row_for(view, slot));

    return rows;
}

const std::vector<graded_row> &graded_once()
{
    static const std::vector<graded_row> rows = graded_rows();

    return rows;
}

std::string graded_line(const ais4104::graded_task &entry, std::string_view spread, const spread_result &tallied, const praxis::evaluation::tolerance_pair &allowed)
{
    return std::format("task {} {} {} over {} cases, worst case {}, residual {:.6e} / {:.6e}, allowed {:.6e} / {:.6e}", entry.task, entry.slot, spread, graded_cases,
                       tallied.worst_case_index, tallied.worst.magnitude, tallied.worst.linear_error_metres, allowed.magnitude, allowed.linear_metres);
}

// An answer of infinity or not-a-number is reported rather than required against. The printed
// algorithms decide a case on an exact predicate -- "if R = I" -- and say nothing about how to
// decide it in floating point, so a faithful reading can pick a test whose boundary sits away from
// the arithmetic the next case performs, and answer non-finitely between the two. That is a
// property of the transcription, not a wrong answer, and the row names the task and the case it was
// found at so it can be read.
std::string non_finite_line(const ais4104::graded_task &entry, std::string_view spread)
{
    return std::format("task {} {} {} answered non-finitely on at least one of {} cases", entry.task, entry.slot, spread, graded_cases);
}

std::string reach_line(const ais4104::graded_task &entry, std::size_t bound_measured_to_cases)
{
    return std::format("task {} {} draws {} cases, past the {} the published bound was measured over", entry.task, entry.slot, graded_cases, bound_measured_to_cases);
}

void warned(const ais4104::graded_task &entry, const graded_row &row)
{
    if(!agreed_throughout(row.bulk, graded_cases))
        WARN(graded_line(entry, "bulk", row.bulk, row.allowed));

    if(!agreed_throughout(row.near_singular, graded_cases))
        WARN(graded_line(entry, "near-singular", row.near_singular, row.allowed));

    if(!row.bulk.finite)
        WARN(non_finite_line(entry, "bulk"));

    if(!row.near_singular.finite)
        WARN(non_finite_line(entry, "near-singular"));

    if(row.bound_measured_to_cases > 0 && graded_cases > row.bound_measured_to_cases)
        WARN(reach_line(entry, row.bound_measured_to_cases));
}

}

TEST_CASE("every graded rigid-motion task is exercised and reports its deviation from praxis's reference", "[.extended][rigid_motion][grading]")
{
    const std::vector<graded_row> &rows = graded_once();
    REQUIRE(rows.size() == ais4104::graded_tasks.size());

    for(std::size_t index = 0; index < rows.size(); ++index)
    {
        const ais4104::graded_task &entry = ais4104::graded_tasks[index];
        const graded_row &row             = rows[index];

        DYNAMIC_SECTION(std::format("task {} - {}", entry.task, entry.slot))
        {
            INFO(graded_line(entry, "bulk", row.bulk, row.allowed));
            INFO(graded_line(entry, "near-singular", row.near_singular, row.allowed));

            warned(entry, row);

            REQUIRE(exercised_throughout(row.bulk, graded_cases));
            REQUIRE(exercised_throughout(row.near_singular, graded_cases));
        }
    }
}

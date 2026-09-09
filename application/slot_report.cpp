#include "slot_report.h"

#include <praxis/rigid_motion/evaluation.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/evaluation/report.h>
#include <praxis/evaluation/slot_evaluation.h>

#include <spdlog/spdlog.h>

#include <array>
#include <format>
#include <string>
#include <vector>
#include <cstddef>

namespace ais4104 {
namespace {

constexpr std::size_t reported_cases = 3;

bool agrees(praxis::evaluation::agreement verdict)
{
    return verdict == praxis::evaluation::agreement::agreed || verdict == praxis::evaluation::agreement::both_refused;
}

slot_state state_of(praxis::evaluation::agreement beside_reference, praxis::evaluation::agreement beside_unwritten)
{
    if(agrees(beside_reference))
        return slot_state::written_and_correct;

    return agrees(beside_unwritten) ? slot_state::unwritten : slot_state::written_and_disagrees;
}

std::string named(const std::vector<classified_slot> &classified, slot_state wanted)
{
    std::string names;
    for(const classified_slot &one : classified)
        if(one.state == wanted)
            names += (names.empty() ? "" : ", ") + std::string(one.name);

    return names.empty() ? std::string("none") : names;
}

}

std::vector<classified_slot> classify_slots(const praxis::rigid_motion::capabilities &motions, std::size_t cases)
{
    const praxis::rigid_motion::capabilities reference = praxis::rigid_motion::baseline();
    const praxis::rigid_motion::capabilities unwritten{};

    const std::array<praxis::evaluation::evaluation_view, 2> beside_reference = praxis::rigid_motion::evaluation_views(motions, reference);
    const std::array<praxis::evaluation::evaluation_view, 2> beside_unwritten = praxis::rigid_motion::evaluation_views(motions, unwritten);

    const praxis::evaluation::evaluation_report by_reference = praxis::evaluation::evaluate(beside_reference, praxis::evaluation::default_seed, cases);
    const praxis::evaluation::evaluation_report by_unwritten = praxis::evaluation::evaluate(beside_unwritten, praxis::evaluation::default_seed, cases);

    std::vector<classified_slot> classified;
    classified.reserve(by_reference.slots.size());
    for(std::size_t index = 0; index < by_reference.slots.size(); ++index)
    {
        const praxis::evaluation::slot_report &reported = by_reference.slots[index];
        classified.push_back({state_of(reported.verdict, by_unwritten.slots[index].verdict), reported.slot});
    }

    return classified;
}

void report_slot_state(const praxis::rigid_motion::capabilities &motions)
{
    const std::vector<classified_slot> classified = classify_slots(motions, reported_cases);

    spdlog::info(std::format("Operations that have not been written yet: {}", named(classified, slot_state::unwritten)));
    spdlog::info(std::format("Operations that are written and disagree with the reference: {}", named(classified, slot_state::written_and_disagrees)));
}

}

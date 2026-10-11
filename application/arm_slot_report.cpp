#include "slot_report.h"
#include "arm_slot_report.h"

#include <praxis/manipulator/evaluation.h>
#include <praxis/manipulator/capabilities.h>

#include <praxis/evaluation/report.h>
#include <praxis/evaluation/slot_evaluation.h>

#include <spdlog/spdlog.h>

#include <span>
#include <array>
#include <format>
#include <string>
#include <vector>
#include <cstddef>

namespace ais4104 {
namespace {

constexpr std::size_t reported_cases = 3;
constexpr std::size_t owned_slots    = 2;

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

praxis::manipulator::capabilities unwritten_arm()
{
    praxis::manipulator::capabilities blank = praxis::manipulator::baseline();
    blank.fk                                = praxis::manipulator::forward_kinematics_ops{};
    blank.dk                                = praxis::manipulator::differential_kinematics_ops{};

    return blank;
}

}

std::vector<classified_slot> classify_arm_slots(const praxis::manipulator::capabilities &arm, std::size_t cases)
{
    const praxis::manipulator::capabilities reference = praxis::manipulator::baseline();
    const praxis::manipulator::capabilities unwritten = unwritten_arm();

    const std::array<praxis::evaluation::evaluation_view, 7> beside_reference = praxis::manipulator::evaluation_views(arm, reference);
    const std::array<praxis::evaluation::evaluation_view, 7> beside_unwritten = praxis::manipulator::evaluation_views(arm, unwritten);

    const praxis::evaluation::evaluation_report by_reference = praxis::evaluation::evaluate(std::span(beside_reference).first(owned_slots), praxis::evaluation::default_seed, cases);
    const praxis::evaluation::evaluation_report by_unwritten = praxis::evaluation::evaluate(std::span(beside_unwritten).first(owned_slots), praxis::evaluation::default_seed, cases);

    std::vector<classified_slot> classified;
    classified.reserve(by_reference.slots.size());
    for(std::size_t index = 0; index < by_reference.slots.size(); ++index)
    {
        const praxis::evaluation::slot_report &reported = by_reference.slots[index];
        classified.push_back({state_of(reported.verdict, by_unwritten.slots[index].verdict), reported.slot});
    }

    return classified;
}

void report_arm_slot_state(const praxis::manipulator::capabilities &arm)
{
    const std::vector<classified_slot> classified = classify_arm_slots(arm, reported_cases);

    spdlog::info(std::format("Operations that have not been written yet: {}", named(classified, slot_state::unwritten)));
    spdlog::info(std::format("Operations that are written and disagree with the reference: {}", named(classified, slot_state::written_and_disagrees)));
    spdlog::info(std::format("Operations that are written and agree with the reference: {}", named(classified, slot_state::written_and_correct)));
}

}

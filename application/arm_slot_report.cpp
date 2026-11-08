#include "slot_report.h"
#include "arm_slot_report.h"

#include <praxis/manipulator/evaluation.h>
#include <praxis/manipulator/capabilities.h>

#include <praxis/trajectory/evaluation.h>
#include <praxis/trajectory/capabilities.h>

#include <praxis/optimization/evaluation.h>
#include <praxis/optimization/capabilities.h>

#include <praxis/evaluation/report.h>
#include <praxis/evaluation/slot_evaluation.h>

#include <praxis/extension/coverage.h>
#include <praxis/extension/slot_set.h>
#include <praxis/extension/descriptor.h>

#include <spdlog/spdlog.h>

#include <span>
#include <array>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <string_view>

namespace ais4104 {
namespace {

constexpr std::size_t reported_cases = 3;

template<std::size_t Views>
struct compared_capabilities
{
    std::array<praxis::evaluation::evaluation_view, Views> beside_reference;
    std::array<praxis::evaluation::evaluation_view, Views> beside_unwritten;
    std::array<praxis::capability_view, Views> described;
};

struct owned_view
{
    praxis::evaluation::evaluation_view beside_reference;
    praxis::evaluation::evaluation_view beside_unwritten;
    std::vector<std::string_view> names;
};

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
    blank.ik                                = praxis::manipulator::inverse_kinematics_ops{};
    blank.robot                             = praxis::manipulator::robot_ops{};

    return blank;
}

praxis::trajectory::capabilities unwritten_shapes()
{
    praxis::trajectory::capabilities blank = praxis::trajectory::baseline();
    blank.time_scaling                     = praxis::trajectory::time_scaling_ops{};
    blank.path                             = praxis::trajectory::path_ops{};
    blank.trajectory                       = praxis::trajectory::trajectory_ops{};

    return blank;
}

template<std::size_t Views, praxis::slot_enumeration Slot>
owned_view owned_at(const compared_capabilities<Views> &compared, std::size_t view, const praxis::basic_slot_set<Slot> &owned)
{
    owned_view taken{compared.beside_reference[view], compared.beside_unwritten[view], {}};
    for(std::size_t index = 0; index < static_cast<std::size_t>(Slot::count); ++index)
        if(owned.contains(static_cast<Slot>(index)))
            taken.names.push_back(praxis::slot_name(compared.described[view], index));

    return taken;
}

std::vector<owned_view> owned_views(const compared_capabilities<7> &arm, const compared_capabilities<4> &shapes, const compared_capabilities<2> &solver, const owned_slots &owned)
{
    std::vector<owned_view> views;
    views.push_back(owned_at(arm, 0, owned.fk));
    views.push_back(owned_at(arm, 1, owned.dk));
    views.push_back(owned_at(arm, 2, owned.ik));
    views.push_back(owned_at(arm, 3, owned.robot));
    views.push_back(owned_at(shapes, 0, owned.time_scaling));
    views.push_back(owned_at(shapes, 1, owned.path));
    views.push_back(owned_at(shapes, 3, owned.trajectory));
    views.push_back(owned_at(solver, 0, owned.solver));
    views.push_back(owned_at(solver, 1, owned.minimizer));

    return views;
}

void classify_view(std::vector<classified_slot> &classified, const owned_view &owned, std::size_t cases)
{
    if(owned.names.empty())
        return;

    const praxis::evaluation::evaluation_report by_reference = praxis::evaluation::evaluate(std::span(&owned.beside_reference, 1), praxis::evaluation::default_seed, cases);
    const praxis::evaluation::evaluation_report by_unwritten = praxis::evaluation::evaluate(std::span(&owned.beside_unwritten, 1), praxis::evaluation::default_seed, cases);

    for(std::size_t index = 0; index < by_reference.slots.size(); ++index)
    {
        const praxis::evaluation::slot_report &reported = by_reference.slots[index];
        if(std::ranges::find(owned.names, reported.slot) != owned.names.end())
            classified.push_back({state_of(reported.verdict, by_unwritten.slots[index].verdict), reported.slot});
    }
}

}

std::vector<classified_slot> classify_arm_slots(const praxis::manipulator::capabilities &arm, const praxis::trajectory::capabilities &shapes,
                                                const praxis::optimization::capabilities &solving, const owned_slots &owned, std::size_t cases)
{
    const praxis::manipulator::capabilities reference         = praxis::manipulator::baseline();
    const praxis::manipulator::capabilities unwritten         = unwritten_arm();
    const praxis::trajectory::capabilities reference_shapes   = praxis::trajectory::baseline();
    const praxis::trajectory::capabilities blank_shapes       = unwritten_shapes();
    const praxis::optimization::capabilities reference_solver = praxis::optimization::baseline();
    const praxis::optimization::capabilities blank_solver{};

    const compared_capabilities<7> arm_views{praxis::manipulator::evaluation_views(arm, reference), praxis::manipulator::evaluation_views(arm, unwritten),
                                             praxis::manipulator::capability_views(arm)};
    const compared_capabilities<4> shape_views{praxis::trajectory::evaluation_views(shapes, reference_shapes), praxis::trajectory::evaluation_views(shapes, blank_shapes),
                                               praxis::trajectory::capability_views(shapes)};
    const compared_capabilities<2> solver_views{praxis::optimization::evaluation_views(solving, reference_solver), praxis::optimization::evaluation_views(solving, blank_solver),
                                                praxis::optimization::capability_views(solving)};

    std::vector<classified_slot> classified;
    for(const owned_view &one : owned_views(arm_views, shape_views, solver_views, owned))
        classify_view(classified, one, cases);

    return classified;
}

void report_arm_slot_state(const praxis::manipulator::capabilities &arm, const praxis::trajectory::capabilities &shapes, const praxis::optimization::capabilities &solving,
                           const owned_slots &owned)
{
    const std::vector<classified_slot> classified = classify_arm_slots(arm, shapes, solving, owned, reported_cases);

    spdlog::info(std::format("Operations that have not been written yet: {}", named(classified, slot_state::unwritten)));
    spdlog::info(std::format("Operations that are written and disagree with the reference: {}", named(classified, slot_state::written_and_disagrees)));
    spdlog::info(std::format("Operations that are written and agree with the reference: {}", named(classified, slot_state::written_and_correct)));
}

}

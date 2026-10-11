#include "kept_chain.h"
#include "arm_windows.h"
#include "arm_keyspace.h"
#include "hand_derived.h"
#include "offered_arms.h"
#include "configuration.h"

#include <praxis/presets/arm.h>
#include <praxis/presets/routes.h>
#include <praxis/presets/screw_table.h>
#include <praxis/presets/arm_scenarios.h>
#include <praxis/presets/arm_registration.h>

#include <praxis/manipulator/compose_arm.h>

#include <praxis/scene/preset.h>
#include <praxis/scene/preset_site.h>

#include <praxis/config/store.h>
#include <praxis/config/binding.h>

#include <spdlog/spdlog.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <filesystem>

namespace ais4104 {
namespace {

using arm_composer = praxis::manipulator::arm_composition (*)(praxis::presets::arm_scenario);

struct arm_routes
{
    praxis::presets::document_route located;
    praxis::presets::document_route written;
    praxis::presets::composed_route announced;
};

struct arm_halves
{
    praxis::manipulator::capabilities arm;
    praxis::trajectory::capabilities shapes;
    praxis::rigid_motion::capabilities motions;
};

struct arm_registration
{
    arm_routes taken;
    arm_halves halves;
    std::vector<std::filesystem::path> roots;
    tool_offset_source offsets;
};

std::string text_at(const praxis::config::document &values, const char *key)
{
    return values.text(key).value_or(std::string());
}

void refused(const praxis::config::location &at, const std::string &why)
{
    spdlog::error(std::format("The document {} offers no preset: {}", at.resolved.string(), why));
}

std::nullopt_t not_offered(const praxis::config::location &at, const std::string &why)
{
    refused(at, why);

    return std::nullopt;
}

std::optional<std::string> offered_name(praxis::scene::preset_registry &registry, const praxis::config::location &at, const praxis::config::outcome &read)
{
    if(read.failure)
        return not_offered(at, read.failure->message);

    const std::string named = text_at(read.values, "preset/name");
    if(named.empty())
        return not_offered(at, "it states no name");

    if(registry.load_preset(named) != nullptr)
        return not_offered(at, std::format("the name {} is already carried", named));

    if(text_at(read.values, "description/path").empty())
        return not_offered(at, "it names no description");

    return named;
}

praxis::manipulator::arm_composition modeling_keeping_nothing(praxis::presets::arm_scenario chosen)
{
    return praxis::presets::arm_windows_modeling(std::move(chosen), praxis::presets::screw_table_source{});
}

arm_composer composer_named(praxis::presets::arm_scenario_kind kind)
{
    if(kind == praxis::presets::arm_scenario_kind::forward_kinematics)
        return &praxis::presets::arm_windows_forward;

    if(kind == praxis::presets::arm_scenario_kind::tooling)
        return &praxis::presets::arm_windows_tooling;

    if(kind == praxis::presets::arm_scenario_kind::velocity_kinematics)
        return &praxis::presets::arm_windows_velocity_kinematics;

    if(kind == praxis::presets::arm_scenario_kind::supplied_chain)
        return &modeling_keeping_nothing;

    return nullptr;
}

arm_composer offered_composer(const praxis::config::location &at, const praxis::config::document &values, std::optional<praxis::presets::arm_scenario_kind> kind)
{
    const arm_composer windows = kind ? composer_named(*kind) : nullptr;
    if(windows == nullptr)
        refused(at, std::format("it names the scenario {}, which is not offered", text_at(values, "preset/scenario")));

    return windows;
}

opened_arm opened_as(praxis::presets::arm_scenario_kind kind, arm_composer windows, const praxis::presets::arm_scenario &machine, const praxis::config::binding &bound,
                     const praxis::config::document &carried, const arm_routes &taken)
{
    if(kind == praxis::presets::arm_scenario_kind::supplied_chain)
        if(std::optional<opened_arm> kept = opened_keeping(machine, carried, taken.written); kept)
            return std::move(*kept);

    return opened_arm{windows(machine), bound, carried, machine};
}

praxis::scene::preset_registry::factory arm_factory(std::string named, praxis::config::location registered, const arm_registration &over, praxis::presets::arm_scenario_kind kind,
                                                    arm_composer windows)
{
    return [named = std::move(named), registered = std::move(registered), over, kind, windows](const praxis::scene::preset_site &site)
    {
        const praxis::config::binding bound{assignment_arm_keyspace(), over.taken.located(registered.given), praxis::config::expectation::partial};
        const praxis::config::outcome carried = praxis::config::load_or_defaults(bound);
        const praxis::presets::arm_scenario machine =
                with_hand_derived_tool(praxis::presets::read_arm(carried.values, over.roots), carried.values, over.halves.motions.frame, over.offsets);
        opened_arm opened                             = opened_as(kind, windows, machine, bound, carried.values, over.taken);
        praxis::manipulator::arm_composition composed = with_offered_pose_frames(
                without_tool_window(with_hand_derived_tool_window(std::move(opened.windows), opened.opening, carried.values, over.offsets), carried.values), kind, carried.values);

        const std::shared_ptr<praxis::scene::preset> built = praxis::presets::arm_preset(site, over.halves.arm, over.halves.shapes, over.halves.motions, opened.opening,
                                                                                         composing(named, kind, opened.opening, carried.values, std::move(composed)));
        if(built != nullptr)
            over.taken.announced(opened.announced, opened.carried);

        return built;
    };
}

std::optional<std::string> register_one(const std::shared_ptr<praxis::scene::preset_registry> &registry, const praxis::config::location &at, const arm_registration &over)
{
    const praxis::config::binding bound{assignment_arm_keyspace(), at, praxis::config::expectation::partial};
    const praxis::config::outcome read = praxis::config::load_or_defaults(bound);

    const std::optional<std::string> named                       = offered_name(*registry, at, read);
    const std::optional<praxis::presets::arm_scenario_kind> kind = named ? praxis::presets::arm_scenario_named(read.values) : std::nullopt;
    const arm_composer windows                                   = named ? offered_composer(at, read.values, kind) : nullptr;
    if(windows == nullptr)
        return std::nullopt;

    registry->register_preset(*named, arm_factory(*named, at, over, *kind, windows));

    return named;
}

}

std::vector<std::string> register_offered_arms(const std::shared_ptr<praxis::scene::preset_registry> &registry, const document_store &mine, const praxis::config::document &values,
                                               const std::shared_ptr<write_back> &writing, const std::filesystem::path &packages, const std::filesystem::path &assets,
                                               const praxis::manipulator::capabilities &arm, const praxis::trajectory::capabilities &shapes,
                                               const praxis::rigid_motion::capabilities &motions, tool_offset_source offsets)
{
    const std::vector<praxis::config::location> reading = preset_locations(values, mine);
    const arm_routes taken{[mine](const std::filesystem::path &named) { return mine.reading(named); }, [mine](const std::filesystem::path &named) { return mine.writing(named); },
                           [writing](const praxis::config::binding &at, const praxis::config::document &carried) { writing->composing(at, carried); }};
    const arm_registration over{taken, {arm, shapes, motions}, {packages, assets}, offsets};

    std::vector<std::string> registered;
    for(const praxis::config::location &at : reading)
    {
        const std::optional<std::string> named = register_one(registry, at, over);
        if(named)
            registered.push_back(*named);
    }

    return registered;
}

}

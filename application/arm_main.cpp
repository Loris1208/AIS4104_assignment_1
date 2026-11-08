#include "arm.h"
#include "shapes.h"
#include "motions.h"
#include "solving.h"
#include "functions.h"
#include "write_back.h"
#include "offered_arms.h"
#include "configuration.h"
#include "document_store.h"
#include "arm_render_loop.h"
#include "arm_slot_report.h"

#include <praxis/presets/optimization.h>

#include <praxis/manipulator/capabilities.h>

#include <praxis/trajectory/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/optimization/problem.h>
#include <praxis/optimization/capabilities.h>

#include <praxis/scene/log_buffer.h>
#include <praxis/scene/visualizer.h>
#include <praxis/scene/preset_registry.h>

#include <praxis/config/store.h>
#include <praxis/config/binding.h>
#include <praxis/config/document.h>

#include <spdlog/spdlog.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <filesystem>
#include <system_error>

#ifndef AIS4104_APPLICATION_CONFIGURATION
    #define AIS4104_APPLICATION_CONFIGURATION "assignment-2.xml"
#endif

#ifndef PRAXIS_PERSISTENT_DIRECTORY
    #define PRAXIS_PERSISTENT_DIRECTORY "."
#endif

namespace ais4104 {
namespace {

constexpr int renderer_unavailable = 3;

constexpr const char *layout_document = "imgui.ini";

template<typename Registered>
std::vector<Registered> joined(std::vector<Registered> first, const std::vector<Registered> &second)
{
    first.insert(first.end(), second.begin(), second.end());

    return first;
}

template<typename Registered>
std::vector<Registered> under_form(const std::vector<Registered> &offered, praxis::optimization::problem_form form)
{
    std::vector<Registered> tagged;
    tagged.reserve(offered.size());
    for(const Registered &one : offered)
        tagged.push_back(one.offered_under({form}));

    return tagged;
}

registered_functions under_their_forms(const offered_function_forms &offered)
{
    using praxis::optimization::problem_form;

    return {joined(under_form(offered.root.curves, problem_form::root), under_form(offered.minimum.curves, problem_form::minimum)),
            joined(under_form(offered.root.surfaces, problem_form::root), under_form(offered.minimum.surfaces, problem_form::minimum))};
}

std::filesystem::path persistent_directory(const std::filesystem::path &beside)
{
    const std::filesystem::path kept = praxis::config::resolve(PRAXIS_PERSISTENT_DIRECTORY, beside).resolved;

    std::error_code failed;
    std::filesystem::create_directories(kept, failed);
    if(failed)
        spdlog::error(std::format("The directory {} could not be created: {}", kept.string(), failed.message()));

    return kept;
}

praxis::config::binding preferences_at(const std::filesystem::path &beside)
{
    return praxis::config::binding{preferences_keyspace(), praxis::config::resolve(persistent_directory(beside) / "praxis-preferences.xml", beside),
                                   praxis::config::expectation::partial};
}

praxis::config::binding application_at(const std::filesystem::path &beside)
{
    return praxis::config::binding{application_keyspace(), praxis::config::resolve(AIS4104_APPLICATION_CONFIGURATION, beside), praxis::config::expectation::complete};
}

document_store own_documents(const praxis::config::binding &bound, const std::filesystem::path &beside)
{
    return {bound.at.resolved.parent_path(), persistent_directory(beside)};
}

std::vector<std::string> announce(std::vector<std::string> registered)
{
    std::string named;
    for(const std::string &one : registered)
        named += (named.empty() ? "" : ", ") + one;

    spdlog::info(std::format("Registered {} presets: {}", registered.size(), named));

    return registered;
}

praxis::config::outcome preferences_read(const praxis::config::binding &prefs)
{
    const praxis::config::outcome preferred = praxis::config::load_or_defaults(prefs);
    praxis::scene::set_reporting_level(preferred_level(preferred.values));

    return preferred;
}

std::vector<std::string> composed_functions(const std::shared_ptr<praxis::scene::preset_registry> &registry, const document_store &mine, const praxis::config::document &values,
                                            const std::shared_ptr<write_back> &writing, const praxis::optimization::capabilities &solving)
{
    constexpr bool checks_derivative   = true;
    const registered_functions offered = under_their_forms(offered_functions());

    return praxis::presets::register_optimization(
            registry, function_locations(values, mine), [mine](const std::filesystem::path &named) { return mine.reading(named); },
            [writing](const praxis::config::binding &at, const praxis::config::document &carried) { writing->composing(at, carried); }, solving, offered.curves, offered.surfaces,
            checks_derivative);
}

std::vector<std::string> composed_arms(const std::shared_ptr<praxis::scene::preset_registry> &registry, const document_store &mine, const praxis::config::document &values,
                                       const std::shared_ptr<write_back> &writing, const std::filesystem::path &beside)
{
    const offered_capabilities offered{offered_arm(), offered_shapes(), offered_motions(), offered_solving()};

    std::vector<std::string> registered      = register_offered_arms(registry, mine, values, writing, beside / "urdf", beside, offered, offered_tool_offsets());
    const std::vector<std::string> functions = composed_functions(registry, mine, values, writing, offered.solving);
    registered.insert(registered.end(), functions.begin(), functions.end());

    const std::vector<std::string> opened = announce(std::move(registered));
    report_arm_slot_state(offered.arm, offered.shapes, offered.solving, offered_owned_slots());

    return opened;
}

int show_application(const std::filesystem::path &beside, const std::shared_ptr<praxis::scene::log_buffer> &messages)
{
    const praxis::config::binding prefs      = preferences_at(beside);
    const praxis::config::outcome preferred  = preferences_read(prefs);
    const praxis::config::binding bound      = application_at(beside);
    const praxis::config::outcome configured = praxis::config::load_or_defaults(bound);

    const document_store mine = own_documents(bound, beside);
    const auto writing        = std::make_shared<write_back>(bound, configured.values, prefs, preferred.values, mine);
    const auto registry       = std::make_shared<praxis::scene::preset_registry>();

    const std::vector<std::string> opened = composed_arms(registry, mine, configured.values, writing, beside);
    static_cast<void>(mine.writing(layout_document));

    const std::optional<praxis::scene::visualizer::geometry> left =
            run_and_report(registry, writing, messages, opened, preferred_geometry(preferred.values), prefs.at.resolved.parent_path());
    if(!left)
        return renderer_unavailable;

    static_cast<void>(praxis::config::save(prefs, preferences_edits(*left, praxis::scene::reporting_level())));

    return 0;
}

}
}

int main(int, char **argv)
{
    const std::filesystem::path beside = std::filesystem::weakly_canonical(std::filesystem::path(argv[0])).parent_path();

    const auto messages = std::make_shared<praxis::scene::log_buffer>(praxis::scene::default_log_capacity);
    praxis::scene::install_log_sink(messages);

    return ais4104::show_application(beside, messages);
}

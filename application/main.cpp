#include "motions.h"
#include "offered.h"
#include "write_back.h"
#include "slot_report.h"
#include "configuration.h"
#include "document_store.h"

#include <praxis/scene/log_buffer.h>
#include <praxis/scene/visualizer.h>
#include <praxis/scene/preset_registry.h>

#include <praxis/config/store.h>
#include <praxis/config/binding.h>
#include <praxis/config/document.h>

#include <praxis/scheduler/task.h>
#include <praxis/scheduler/strand.h>
#include <praxis/scheduler/overrun.h>
#include <praxis/scheduler/scheduler.h>

#include <spdlog/spdlog.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <exception>
#include <filesystem>
#include <system_error>

#ifndef AIS4104_PREVIEW_FRAMES
    #define AIS4104_PREVIEW_FRAMES 0
#endif

#ifndef AIS4104_APPLICATION_CONFIGURATION
    #define AIS4104_APPLICATION_CONFIGURATION "rigid-motions.xml"
#endif

#ifndef PRAXIS_PERSISTENT_DIRECTORY
    #define PRAXIS_PERSISTENT_DIRECTORY "."
#endif

namespace ais4104 {
namespace {

constexpr int frames_to_draw       = AIS4104_PREVIEW_FRAMES;
constexpr int renderer_unavailable = 3;

constexpr const char *layout_document = "imgui.ini";

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

praxis::scheduler::task_handle draw_until_done(praxis::scene::visualizer &view, praxis::scheduler::scheduler &loop)
{
    return view.executor().every(praxis::scheduler::every_step, praxis::scheduler::overrun::drop,
                                 [&view, &loop, drawn = 0, closing = false](praxis::scheduler::step_delta) mutable
                                 {
                                     if(closing)
                                         return;

                                     const bool continuing = view.render_once();
                                     ++drawn;
                                     if(continuing && (frames_to_draw <= 0 || drawn < frames_to_draw))
                                         return;

                                     closing = true;
                                     view.release_preset([&loop] { loop.stop(); });
                                 });
}

praxis::scene::visualizer::geometry run_until_closed(const std::shared_ptr<praxis::scene::preset_registry> &registry, const std::shared_ptr<write_back> &writing,
                                                     const std::shared_ptr<praxis::scene::log_buffer> &messages, const std::vector<std::string> &opened,
                                                     const praxis::scene::visualizer::geometry &window, const std::filesystem::path &root)
{
    praxis::scheduler::scheduler loop(praxis::scheduler::default_workers());
    praxis::scene::visualizer view(registry, loop, {.view = praxis::scene::visualizer::projection::perspective, .messages = messages, .window = window, .root = root});
    install_write_back(view, writing);
    if(!opened.empty())
        view.load_preset(opened.front());

    const praxis::scheduler::task_handle frame = draw_until_done(view, loop);

    loop.run();

    return view.window_geometry();
}

std::optional<praxis::scene::visualizer::geometry> run_and_report(const std::shared_ptr<praxis::scene::preset_registry> &registry, const std::shared_ptr<write_back> &writing,
                                                                  const std::shared_ptr<praxis::scene::log_buffer> &messages, const std::vector<std::string> &opened,
                                                                  const praxis::scene::visualizer::geometry &window, const std::filesystem::path &root)
{
    try
    {
        return run_until_closed(registry, writing, messages, opened, window, root);
    }
    catch(const std::exception &failed)
    {
        spdlog::error(std::format("The renderer could not be started, so there is nothing to show: {}", failed.what()));
    }
    catch(...)
    {
        spdlog::error("The renderer could not be started, so there is nothing to show");
    }

    return std::nullopt;
}

praxis::config::outcome preferences_read(const praxis::config::binding &prefs)
{
    const praxis::config::outcome preferred = praxis::config::load_or_defaults(prefs);
    praxis::scene::set_reporting_level(preferred_level(preferred.values));

    return preferred;
}

int show_application(const std::filesystem::path &beside, const std::shared_ptr<praxis::scene::log_buffer> &messages)
{
    const praxis::config::binding prefs      = preferences_at(beside);
    const praxis::config::outcome preferred  = preferences_read(prefs);
    const praxis::config::binding bound      = application_at(beside);
    const praxis::config::outcome configured = praxis::config::load_or_defaults(bound);

    const document_store mine                        = own_documents(bound, beside);
    const auto writing                               = std::make_shared<write_back>(bound, configured.values, prefs, preferred.values, mine);
    const auto registry                              = std::make_shared<praxis::scene::preset_registry>();
    const praxis::rigid_motion::capabilities motions = offered_motions();

    const std::vector<std::string> opened = announce(register_offered(registry, mine, configured.values, writing, motions));
    report_slot_state(motions);
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

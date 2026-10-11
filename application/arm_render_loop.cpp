#include "write_back.h"
#include "arm_render_loop.h"

#include <praxis/scene/log_buffer.h>
#include <praxis/scene/visualizer.h>
#include <praxis/scene/preset_registry.h>

#include <praxis/scheduler/task.h>
#include <praxis/scheduler/strand.h>
#include <praxis/scheduler/overrun.h>
#include <praxis/scheduler/scheduler.h>

#include <spdlog/spdlog.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <exception>
#include <filesystem>

#ifndef AIS4104_PREVIEW_FRAMES
    #define AIS4104_PREVIEW_FRAMES 0
#endif

namespace ais4104 {
namespace {

constexpr int frames_to_draw = AIS4104_PREVIEW_FRAMES;

bool showing_next(praxis::scene::visualizer &view, const std::vector<std::string> &opened, std::size_t &shown)
{
    // An empty list asked for no preset, so nothing being loaded here is not a preset that failed.
    if(opened.empty())
        return false;

    if(!view.is_preset_loaded())
    {
        spdlog::error(std::format("The preset {} did not load, so the preview stops here", opened[shown]));

        return false;
    }

    if(shown + 1 >= opened.size())
        return false;

    ++shown;
    view.load_preset(opened[shown]);

    return true;
}

praxis::scheduler::task_handle draw_until_done(praxis::scene::visualizer &view, praxis::scheduler::scheduler &loop, const std::vector<std::string> &opened)
{
    return view.executor().every(praxis::scheduler::every_step, praxis::scheduler::overrun::drop,
                                 [&view, &loop, &opened, shown = std::size_t(0), drawn = 0, closing = false](praxis::scheduler::step_delta) mutable
                                 {
                                     if(closing)
                                         return;

                                     const bool continuing = view.render_once();
                                     ++drawn;
                                     if(continuing && (frames_to_draw <= 0 || drawn < frames_to_draw))
                                         return;

                                     drawn = 0;
                                     if(continuing && frames_to_draw > 0 && showing_next(view, opened, shown))
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

    const praxis::scheduler::task_handle frame = draw_until_done(view, loop, opened);

    loop.run();

    return view.window_geometry();
}

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

}

#include "arm_windows.h"

#include <praxis/manipulator/compose_arm.h>
#include <praxis/manipulator/tool_window.h>
#include <praxis/manipulator/arm_snapshot.h>
#include <praxis/manipulator/loadable_robot_stencil.h>

#include <praxis/scene/imgui_window.h>

#include <praxis/config/document.h>

#include <spdlog/spdlog.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

namespace ais4104 {
namespace {

using composed_windows = std::vector<std::shared_ptr<praxis::scene::imgui_window>>;

// The frames the arm is given to publish the tool offset praxis posts through its command queue at
// composition.
constexpr int frames_to_publish = 60;

bool tool_window_opens(const praxis::config::document &values)
{
    const auto opens = values.flag("tool_window/opens");

    return opens ? *opens : true;
}

bool hangs_a_tool(const praxis::manipulator::loadable_robot_stencil &stencil)
{
    return stencil.attached_at(praxis::manipulator::flange_attachment::tool) != nullptr;
}

void report_seat(const std::string &preset, const std::shared_ptr<const praxis::manipulator::arm_snapshot> &seen)
{
    if(seen == nullptr || !seen->tool_offset_known)
    {
        spdlog::error(std::format("The tool center point of {} is not seated: no Tool window opens and the arm had published no tool offset of its own within {} frames", preset,
                                  frames_to_publish));
        return;
    }

    spdlog::info(std::format("The tool center point of {} is seated by praxis with no Tool window, at ({}, {}, {}) m from where it is attached", preset, seen->tool_offset(0, 3),
                             seen->tool_offset(1, 3), seen->tool_offset(2, 3)));
}

// Reads the arm's snapshot each frame until it holds a tool offset of the arm's own or the frames
// run out, then reports once. It draws nothing.
class seat_report : public praxis::scene::imgui_window
{
public:
    seat_report(std::string preset, praxis::manipulator::arm_reader seen)
            : imgui_window("Tool seat report")
            , m_preset(std::move(preset))
            , m_seen(std::move(seen))
    {
    }

    void render() override
    {
        if(m_reported)
            return;

        const std::shared_ptr<const praxis::manipulator::arm_snapshot> seen = m_seen.read();
        if((seen == nullptr || !seen->tool_offset_known) && ++m_waited < frames_to_publish)
            return;

        report_seat(m_preset, seen);
        m_reported = true;
    }

private:
    std::string m_preset;
    praxis::manipulator::arm_reader m_seen;
    int m_waited    = 0;
    bool m_reported = false;
};

}

bool is_tool_window(const std::shared_ptr<praxis::scene::imgui_window> &window)
{
    return std::dynamic_pointer_cast<praxis::manipulator::tool_window>(window) != nullptr;
}

bool is_seat_report(const std::shared_ptr<praxis::scene::imgui_window> &window)
{
    return std::dynamic_pointer_cast<seat_report>(window) != nullptr;
}

praxis::manipulator::arm_composition without_tool_window(praxis::manipulator::arm_composition composed, const praxis::config::document &values)
{
    if(tool_window_opens(values))
        return composed;

    composed.windows = [preset = values.text("preset/name").value_or(std::string()), inner = std::move(composed.windows)](const praxis::manipulator::arm_window_inputs &built)
    {
        composed_windows opened = inner(built);
        if(opened.empty())
            return opened;

        std::erase_if(opened, is_tool_window);
        if(hangs_a_tool(built.stencil))
            opened.push_back(std::make_shared<seat_report>(preset, built.seen));
        else
            spdlog::error(std::format("The tool center point of {} is not seated: no Tool window opens and praxis hangs no tool at the flange, as the document's tool is inactive "
                                      "or its model did not load",
                                      preset));

        return opened;
    };

    return composed;
}

}

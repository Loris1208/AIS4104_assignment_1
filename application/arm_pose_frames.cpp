#include "arm_windows.h"

#include <praxis/presets/arm_scenarios.h>

#include <praxis/manipulator/compose_arm.h>
#include <praxis/manipulator/pose_readout.h>

#include <praxis/scene/imgui_window.h>
#include <praxis/scene/labeled_value_window.h>

#include <praxis/config/document.h>

#include <spdlog/spdlog.h>

#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <string_view>

namespace ais4104 {
namespace {

using composed_windows = std::vector<std::shared_ptr<praxis::scene::imgui_window>>;
using offered_frames   = praxis::manipulator::pose_readout::offered_frames;

struct named_frames
{
    std::string_view name;
    offered_frames offered;
};

constexpr std::array<named_frames, 3> frame_names{{
        {"flange", offered_frames::flange},
        {"tool", offered_frames::tool},
        {"both", offered_frames::both},
}};

std::string frames_named(const praxis::config::document &values)
{
    return values.text("pose_window/frames").value_or(std::string("both"));
}

offered_frames offered_for(std::string_view name)
{
    const auto named = std::ranges::find(frame_names, name, &named_frames::name);

    return named == frame_names.end() ? offered_frames::both : named->offered;
}

bool composes_published_pose(praxis::presets::arm_scenario_kind kind)
{
    return kind == praxis::presets::arm_scenario_kind::forward_kinematics || kind == praxis::presets::arm_scenario_kind::tooling;
}

bool is_pose_window(const std::shared_ptr<praxis::scene::imgui_window> &window)
{
    return window != nullptr && window->display_name() == "Pose" && std::dynamic_pointer_cast<praxis::scene::labeled_value_window>(window) != nullptr;
}

void narrow_pose_window(composed_windows &opened, const praxis::manipulator::arm_window_inputs &built, const std::string &preset, const std::string &frames)
{
    const auto pose = std::ranges::find_if(opened, is_pose_window);
    if(pose == opened.end())
    {
        spdlog::error(std::format("The preset {} opened no window named Pose, so no Pose window reads only the {} frame", preset, frames));
        return;
    }

    *pose = praxis::manipulator::compose_pose_readout("Pose", built.seen, built.frames, built.inert, offered_for(frames));
    spdlog::info(std::format("The Pose window of {} reads only the {} frame", preset, frames));
}

}

praxis::manipulator::arm_composition with_offered_pose_frames(praxis::manipulator::arm_composition composed, praxis::presets::arm_scenario_kind kind,
                                                              const praxis::config::document &values)
{
    std::string frames = frames_named(values);
    if(offered_for(frames) == offered_frames::both)
        return composed;

    std::string preset = values.text("preset/name").value_or(std::string());
    if(!composes_published_pose(kind))
    {
        spdlog::error(std::format("The document of {} sets pose_window/frames to {}, which only a forward kinematics or tooling preset reads", preset, frames));
        return composed;
    }

    composed.windows = [preset = std::move(preset), frames = std::move(frames), inner = std::move(composed.windows)](const praxis::manipulator::arm_window_inputs &built)
    {
        composed_windows opened = inner(built);
        if(!opened.empty())
            narrow_pose_window(opened, built, preset, frames);

        return opened;
    };

    return composed;
}

}

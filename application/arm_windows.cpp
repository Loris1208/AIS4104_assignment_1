#include "arm_windows.h"
#include "jacobian_jog_window.h"

#include <praxis/presets/arm.h>

#include <praxis/manipulator/compose_arm.h>
#include <praxis/manipulator/edited_pose.h>
#include <praxis/manipulator/arm_snapshot.h>
#include <praxis/manipulator/tool_jog_window.h>
#include <praxis/manipulator/screw_jog_window.h>
#include <praxis/manipulator/task_space_window.h>
#include <praxis/manipulator/world_object_window.h>

#include <praxis/scene/imgui_window.h>
#include <praxis/scene/labeled_value_window.h>

#include <praxis/rigid_motion/angles.h>

#include <spdlog/spdlog.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <functional>

namespace ais4104 {
namespace {

using composed_windows = std::vector<std::shared_ptr<praxis::scene::imgui_window>>;
using shared_pose      = std::shared_ptr<praxis::manipulator::edited_pose>;

void append_tool_jog(composed_windows &opened, const praxis::manipulator::arm_window_inputs &built, const praxis::presets::arm_scenario &state, const shared_pose &edited)
{
    opened.push_back(std::make_shared<praxis::manipulator::tool_jog_window>("Tool frame motion", built.seen, built.arm, built.frames, edited, state.tool_jog,
                                                                            praxis::presets::window_paths::tool_jog, praxis::presets::tool_pose_path));
}

void append_pose_target(composed_windows &opened, const praxis::manipulator::arm_window_inputs &built, const praxis::presets::arm_scenario &state, const shared_pose &edited)
{
    opened.push_back(std::make_shared<praxis::manipulator::task_space_window>("Pose target", built.seen, built.arm, built.frames, edited, state.task_space,
                                                                              praxis::presets::window_paths::task_space, praxis::presets::tool_pose_path));
}

void append_task_jogs(composed_windows &opened, const praxis::manipulator::arm_window_inputs &built, const praxis::presets::arm_scenario &state, const shared_pose &edited)
{
    append_pose_target(opened, built, state, edited);
    opened.push_back(std::make_shared<praxis::manipulator::screw_jog_window>("Screw displacement", built.seen, built.arm, built.frames, edited, state.screw_jog,
                                                                             praxis::presets::window_paths::screw_jog, praxis::presets::tool_pose_path));
}

void append_tooling_jogs(composed_windows &opened, const praxis::manipulator::arm_window_inputs &built, const praxis::presets::arm_scenario &state, const shared_pose &edited,
                         bool pose_target)
{
    if(pose_target)
        append_pose_target(opened, built, state, edited);
    append_tool_jog(opened, built, state, edited);
}

bool opens_pose_target(const std::string &named, praxis::presets::arm_scenario_kind kind, const praxis::config::document &values)
{
    const auto opens = values.flag("pose_target_window/opens");
    const bool asked = opens ? *opens : false;
    if(asked && kind != praxis::presets::arm_scenario_kind::tooling)
    {
        spdlog::error(std::format("The document of {} sets pose_target_window/opens, which only a tooling preset reads", named));
        return false;
    }

    return asked;
}

praxis::scene::readout joint_positions(const std::shared_ptr<const praxis::manipulator::arm_snapshot> &seen)
{
    if(seen == nullptr)
        return praxis::scene::readout{"The arm has published nothing yet.", {}};

    praxis::scene::readout shown;
    for(Eigen::Index joint = 0; joint < seen->joints.size(); ++joint)
        shown.rows.push_back({praxis::scene::labeled_value{static_cast<float>(praxis::to_degrees(seen->joints[joint])), std::format("j{}", joint + 1)}});

    return shown;
}

std::shared_ptr<praxis::scene::imgui_window> joint_position_window(const praxis::manipulator::arm_window_inputs &built)
{
    return std::make_shared<praxis::scene::labeled_value_window>("Joint positions", nullptr, [seen = built.seen] { return joint_positions(seen.read()); });
}

// An empty list is the inner composer having declined.
void append_jogs(composed_windows &opened, const praxis::manipulator::arm_window_inputs &built, const praxis::presets::arm_scenario &state, praxis::presets::arm_scenario_kind kind,
                 bool pose_target)
{
    if(opened.empty())
        return;

    if(kind == praxis::presets::arm_scenario_kind::supplied_chain)
        return;

    const shared_pose edited = std::make_shared<praxis::manipulator::edited_pose>(state.tool_pose);
    if(kind == praxis::presets::arm_scenario_kind::tooling)
        return append_tooling_jogs(opened, built, state, edited, pose_target);

    append_task_jogs(opened, built, state, edited);
    if(kind == praxis::presets::arm_scenario_kind::velocity_kinematics)
    {
        opened.push_back(std::make_shared<jacobian_jog_window>("Jacobian jog", built.seen, built.arm, built.chain));
        opened.push_back(joint_position_window(built));
    }
    else if(kind == praxis::presets::arm_scenario_kind::forward_kinematics)
        append_tool_jog(opened, built, state, edited);
}

// A tooling document that names no world-object model draws none and offers no window over it.
bool draws_no_world(praxis::presets::arm_scenario_kind kind, const praxis::presets::arm_scenario &state)
{
    return kind == praxis::presets::arm_scenario_kind::tooling && state.world_object.model_path.empty();
}

bool is_world_object_window(const std::shared_ptr<praxis::scene::imgui_window> &window)
{
    return std::dynamic_pointer_cast<praxis::manipulator::world_object_window>(window) != nullptr;
}

}

praxis::manipulator::arm_composition composing(std::string named, praxis::presets::arm_scenario_kind kind, praxis::presets::arm_scenario state, const praxis::config::document &values,
                                               praxis::manipulator::arm_composition composed)
{
    const bool worldless = draws_no_world(kind, state);
    if(worldless)
        composed.draws_world = false;

    const bool pose_target = opens_pose_target(named, kind, values);
    composed.windows =
            [named = std::move(named), kind, worldless, pose_target, state = std::move(state), inner = std::move(composed.windows)](const praxis::manipulator::arm_window_inputs &built)
    {
        composed_windows opened = inner(built);
        if(worldless)
            std::erase_if(opened, is_world_object_window);
        append_jogs(opened, built, state, kind, pose_target);
        spdlog::info(std::format("The preset {} opened {} windows", named, std::ranges::count_if(opened, std::not_fn(is_seat_report))));

        return opened;
    };

    return composed;
}

}

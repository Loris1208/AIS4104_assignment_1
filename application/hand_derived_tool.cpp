#include "arm.h"
#include "arm_windows.h"
#include "hand_derived.h"

#include "modeling/chains.h"

#include <praxis/presets/arm.h>

#include <praxis/manipulator/compose_arm.h>
#include <praxis/manipulator/tool_window.h>

#include <praxis/rigid_motion/angles.h>
#include <praxis/rigid_motion/axis_order.h>
#include <praxis/rigid_motion/capabilities.h>

#include <praxis/scene/imgui_window.h>

#include <praxis/config/writer.h>
#include <praxis/config/document.h>

#include <spdlog/spdlog.h>

#include <Eigen/LU>
#include <Eigen/Core>

#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <string_view>

namespace ais4104 {
namespace {

struct tool_entry
{
    std::string_view named;
    std::string_view function;
    std::string_view frame;
    Eigen::Matrix4d (*derive)(const praxis::rigid_motion::frame_ops &frames);
};

constexpr std::array<tool_entry, 2> tool_entries{{
        {"kr6r900sixx_curved_rod", "hardcoded_kr6r900sixx_curved_rod_tool_offset", "tool0", &modeling::hardcoded_kr6r900sixx_curved_rod_tool_offset},
        {"kr6r900sixx_humerus", "hardcoded_kr6r900sixx_humerus_tool_offset", "tool0", &modeling::hardcoded_kr6r900sixx_humerus_tool_offset},
}};

std::string tool_name_in(const praxis::config::document &values)
{
    return values.text("hand_derived/tool").value_or(std::string());
}

const tool_entry *tool_named(std::string_view named)
{
    const auto found = std::ranges::find(tool_entries, named, &tool_entry::named);

    return found == tool_entries.end() ? nullptr : &*found;
}

const tool_entry *tool_read_by(const praxis::config::document &values, tool_offset_source offsets)
{
    return offsets == tool_offset_source::hand_derived ? tool_named(tool_name_in(values)) : nullptr;
}

void report_unknown_tool(const praxis::config::document &values)
{
    const std::string named = tool_name_in(values);
    if(!named.empty() && tool_named(named) == nullptr)
        spdlog::error(std::format("The document names the hand-derived tool {}, which no function in modules/modeling/chains.cpp answers", named));
}

bool keys_kinematics(const praxis::config::edit &offered)
{
    return offered.key.starts_with(std::format("{}/kinematics/", praxis::presets::window_paths::tool));
}

class hand_derived_tool_window : public praxis::manipulator::tool_window
{
public:
    using praxis::manipulator::tool_window::tool_window;

    std::vector<praxis::config::edit> settings_edits(const praxis::config::document &carried) const override
    {
        std::vector<praxis::config::edit> offered = tool_window::settings_edits(carried);
        std::erase_if(offered, keys_kinematics);

        return offered;
    }
};

std::shared_ptr<praxis::scene::imgui_window> opened_tool_window(const praxis::manipulator::arm_window_inputs &built, const praxis::manipulator::tool_window::settings &state,
                                                                const std::vector<std::filesystem::path> &roots)
{
    return std::make_shared<hand_derived_tool_window>("Tool", built.stencil, built.seen, built.arm, built.frames, state, praxis::presets::window_paths::tool, roots);
}

// The Frobenius distance of R^T R from the identity past which a rotation block is refused.
constexpr double rotation_rigidity = 1e-6;

struct tool_seat
{
    Eigen::Vector3f degrees = Eigen::Vector3f::Zero();
    Eigen::Vector3f offset  = Eigen::Vector3f::Zero();
};

std::optional<std::string_view> refused_because(const Eigen::Matrix4d &answered)
{
    const Eigen::Matrix3d rotation = answered.topLeftCorner<3, 3>();
    if(!answered.allFinite())
        return "that is not finite";
    if(answered.row(3) != Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0))
        return "whose bottom row is not (0, 0, 0, 1)";
    if((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > rotation_rigidity || rotation.determinant() <= 0.0)
        return "whose rotation block is not a rotation";

    return std::nullopt;
}

tool_seat seated(const tool_entry &entry, const std::string &preset, const Eigen::Matrix4d &answered, praxis::axis_order order)
{
    const Eigen::Vector3d radians = praxis::rigid_motion::baseline().frame.euler_from_rotation_matrix(answered.topLeftCorner<3, 3>(), order);
    const tool_seat seat{(radians * praxis::degrees_per_radian).cast<float>(), answered.block<3, 1>(0, 3).cast<float>()};

    spdlog::info(std::format("The tool center point of {} is read from {}() in modules/modeling/chains.cpp: ({}, {}, {}) m in {}, turned {} ({}, {}, {}) degrees", preset,
                             entry.function, answered(0, 3), answered(1, 3), answered(2, 3), entry.frame, praxis::axis_order_labels()[static_cast<std::size_t>(order)], seat.degrees.x(),
                             seat.degrees.y(), seat.degrees.z()));
    return seat;
}

tool_seat seat_read(const tool_entry &entry, const praxis::rigid_motion::frame_ops &frames, const std::string &preset, praxis::axis_order order)
{
    const Eigen::Matrix4d answered = entry.derive(frames);
    if(answered.isZero())
    {
        spdlog::info(std::format("The tool center point of {} has not been derived yet: {}() in modules/modeling/chains.cpp returns no transformation, so it stands at {}", preset,
                                 entry.function, entry.frame));
        return tool_seat{};
    }
    if(const std::optional<std::string_view> why = refused_because(answered))
    {
        spdlog::error(std::format("The tool center point of {} cannot be placed: {}() in modules/modeling/chains.cpp returns a transformation {}, so it stands at {}", preset,
                                  entry.function, *why, entry.frame));
        return tool_seat{};
    }

    return seated(entry, preset, answered, order);
}

}

praxis::presets::arm_scenario with_hand_derived_tool(praxis::presets::arm_scenario machine, const praxis::config::document &values, const praxis::rigid_motion::frame_ops &frames,
                                                     tool_offset_source offsets)
{
    report_unknown_tool(values);

    const tool_entry *const entry = tool_read_by(values, offsets);
    if(entry == nullptr)
        return machine;

    const tool_seat seat                  = seat_read(*entry, frames, values.text("preset/name").value_or(std::string()), machine.tool.kinematics_euler_order);
    machine.tool.kinematics_euler_degrees = seat.degrees;
    machine.tool.kinematics_offset        = seat.offset;

    return machine;
}

praxis::manipulator::arm_composition with_hand_derived_tool_window(praxis::manipulator::arm_composition composed, const praxis::presets::arm_scenario &machine,
                                                                   const praxis::config::document &values, tool_offset_source offsets)
{
    if(tool_read_by(values, offsets) == nullptr)
        return composed;

    composed.windows = [tool = machine.tool, roots = machine.model_roots, inner = std::move(composed.windows)](const praxis::manipulator::arm_window_inputs &built)
    {
        std::vector<std::shared_ptr<praxis::scene::imgui_window>> opened = inner(built);
        if(std::ranges::none_of(opened, is_tool_window))
            return opened;

        std::ranges::replace_if(opened, is_tool_window, opened_tool_window(built, tool, roots));

        return opened;
    };

    return composed;
}

}

#include "arm_keyspace.h"

#include <praxis/presets/arm.h>
#include <praxis/presets/arm_registration.h>

#include <praxis/manipulator/control_mode.h>
#include <praxis/manipulator/tool_jog_window.h>
#include <praxis/manipulator/screw_jog_window.h>
#include <praxis/manipulator/task_space_window.h>
#include <praxis/manipulator/joint_control_window.h>
#include <praxis/manipulator/control_configuration.h>

#include <praxis/config/writer.h>
#include <praxis/config/declaration.h>

#include <span>
#include <string>
#include <vector>
#include <utility>
#include <iterator>
#include <algorithm>

namespace ais4104 {
namespace {

std::vector<praxis::config::edit> opened_in_preview()
{
    namespace manipulator = praxis::manipulator;
    using paths           = praxis::presets::window_paths;
    using manipulator::control_mode;

    manipulator::task_space_window::settings task_space;
    task_space.mode = control_mode::preview;

    std::vector<praxis::config::edit> written = manipulator::write_joint_control(manipulator::joint_control_window::settings{control_mode::preview}, paths::joint_control);
    std::ranges::copy(manipulator::write_task_space(task_space, paths::task_space), std::back_inserter(written));
    std::ranges::copy(manipulator::write_tool_jog(manipulator::tool_jog_window::settings{control_mode::preview}, paths::tool_jog), std::back_inserter(written));
    std::ranges::copy(manipulator::write_screw_jog(manipulator::screw_jog_window::settings{control_mode::preview}, paths::screw_jog), std::back_inserter(written));

    return written;
}

void redeclare(praxis::config::declaration &shape, const praxis::config::node &declared, std::span<const praxis::config::edit> replacing)
{
    const auto replaced  = std::ranges::find(replacing, declared.path, &praxis::config::edit::key);
    std::string fallback = replaced == replacing.end() ? declared.fallback : replaced->value;

    if(declared.shape == praxis::config::node_kind::group)
        shape.group(declared.path);
    else if(declared.shape == praxis::config::node_kind::collection)
        shape.collection(declared.path, declared.identity);
    else if(!declared.allowed.empty())
        shape.choice(declared.path, declared.allowed, std::move(fallback));
    else
        shape.field(declared.path, declared.kind, std::move(fallback));
}

}

praxis::config::declaration assignment_arm_keyspace()
{
    const praxis::config::declaration praxis_keyspace = praxis::presets::arm_keyspace();
    const std::vector<praxis::config::edit> preview   = opened_in_preview();

    praxis::config::declaration shape(praxis_keyspace.space());
    for(const praxis::config::node &declared : praxis_keyspace.nodes())
        redeclare(shape, declared, preview);

    shape.group("hand_derived");
    shape.field("hand_derived/tool", praxis::config::field_kind::text, "");
    shape.group("tool_window");
    shape.field("tool_window/opens", praxis::config::field_kind::flag, "true");
    shape.group("pose_window");
    shape.choice("pose_window/frames", {"flange", "tool", "both"}, "both");
    shape.group("pose_target_window");
    shape.field("pose_target_window/opens", praxis::config::field_kind::flag, "false");

    return shape;
}

}

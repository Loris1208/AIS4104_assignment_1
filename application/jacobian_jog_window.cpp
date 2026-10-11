#include "jacobian_jog_window.h"
#include "jacobian_displacement.h"

#include <praxis/manipulator/types.h>
#include <praxis/manipulator/kinematics.h>
#include <praxis/manipulator/scene_robot.h>
#include <praxis/manipulator/option_widgets.h>
#include <praxis/manipulator/robot_controller.h>

#include <praxis/extension/refusal.h>

#include <praxis/compat/expected.h>

#include <spdlog/spdlog.h>

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <format>
#include <memory>
#include <string>
#include <cstddef>
#include <utility>
#include <optional>

namespace ais4104 {
namespace {

constexpr const char *twist_instruction = "Sliders set a rate. The arm moves while a slider is held away from center.";

constexpr std::array<const char *, 6> twist_labels{"wx", "wy", "wz", "vx", "vy", "vz"};

constexpr std::array<const char *, 3> frame_labels{"World at tool", "Space", "Body"};

constexpr std::array<const char *, 3> jacobian_slots{"dk.space_jacobian", "dk.space_jacobian", "dk.body_jacobian"};

// The slot the world-at-tool frame asks the tool centre point of. The other two frames ask none.
constexpr const char *tool_point_slot = "fk.forward_kinematics";

const char *refusal_name(praxis::refusal reason)
{
    switch(reason)
    {
        case praxis::refusal::unsupported_input:
            return "unsupported input";
        case praxis::refusal::degenerate:
            return "degenerate";
        case praxis::refusal::no_solution:
            return "no solution";
        case praxis::refusal::not_implemented:
            return "not implemented";
    }

    return "unclassified";
}

const char *slot_named(jog_frame frame)
{
    return jacobian_slots[static_cast<std::size_t>(frame)];
}

const praxis::expected<praxis::manipulator::jacobian, praxis::refusal> &published(const praxis::manipulator::arm_snapshot &seen, jog_frame frame)
{
    return frame == jog_frame::body ? seen.body_jacobian : seen.space_jacobian;
}

praxis::expected<praxis::manipulator::jacobian, praxis::refusal> asked_of(const praxis::manipulator::kinematics &solver, jog_frame frame, const praxis::manipulator::joint_vector &at)
{
    return frame == jog_frame::body ? solver.body_jacobian(at) : solver.space_jacobian(at);
}

// A refusal a held slider produces once per frame is one episode, not one event per tick, and the
// message log has a fixed capacity the start-up slot report also lives in.
void refuse_once(praxis::manipulator::robot_controller &control, const char *slot, praxis::refusal reason, std::atomic<std::optional<praxis::refusal>> &reported)
{
    if(reported.exchange(reason) == reason)
        return;

    control.report_refusal(slot, reason, praxis::refusal_standing::composition_wide);
}

void step_once(praxis::manipulator::robot_controller &control, praxis::manipulator::scene_robot &driven, jog_frame frame, const praxis::twist &asked,
               const praxis::manipulator::joint_limits &within, praxis::scheduler::step_delta elapsed, std::atomic<std::optional<praxis::refusal>> &reported)
{
    const praxis::manipulator::joint_vector at = driven.joint_positions();

    const praxis::expected<praxis::manipulator::jacobian, praxis::refusal> answered = asked_of(driven.solver(), frame, at);
    if(!answered)
        return refuse_once(control, slot_named(frame), answered.error(), reported);

    praxis::manipulator::jacobian of = *answered;
    if(frame == jog_frame::world_at_tool)
    {
        // Taken at the configuration the Jacobian was taken at, so no second forward solve enters.
        const praxis::expected<Eigen::Vector3d, praxis::refusal> point = driven.tool_position();
        if(!point)
            return refuse_once(control, tool_point_slot, point.error(), reported);

        of = referenced_to(of, *point);
    }

    const praxis::expected<praxis::manipulator::joint_vector, praxis::refusal> commanded = jogged_configuration(of, asked, at, within, elapsed);
    if(!commanded)
        return refuse_once(control, slot_named(frame), commanded.error(), reported);

    reported.store(std::nullopt);
    control.preview_joint_configuration(*commanded);
}

std::string refused_by(const char *slot, praxis::refusal reason)
{
    return std::format("{} has not been written yet and refuses as {}; the sliders name a twist and the arm stays where it is", slot, refusal_name(reason));
}

std::optional<std::string> outcome_of(const praxis::manipulator::arm_snapshot *seen, jog_frame frame)
{
    if(seen == nullptr)
        return "The arm has published nothing yet, so the sliders drive nothing";

    const praxis::expected<praxis::manipulator::jacobian, praxis::refusal> &answered = published(*seen, frame);
    if(!answered)
        return refused_by(slot_named(frame), answered.error());

    if(frame == jog_frame::world_at_tool && !seen->tool_position)
        return refused_by(tool_point_slot, seen->tool_position.error());

    return std::nullopt;
}

}

bool render_twist_sliders(Eigen::Vector<float, 6> &twist)
{
    ImGui::TextWrapped("%s", twist_instruction);

    bool held = false;
    for(std::size_t slider = 0; slider < twist_labels.size(); ++slider)
    {
        ImGui::SliderFloat(twist_labels[slider], &twist[static_cast<Eigen::Index>(slider)], -twist_slider_bound, twist_slider_bound);
        held |= ImGui::IsItemActive();
    }

    if(ImGui::Button("Reset"))
        twist = Eigen::Vector<float, 6>::Zero();

    return held;
}

jacobian_jog_window::jacobian_jog_window(std::string name, praxis::manipulator::arm_reader seen, std::weak_ptr<praxis::manipulator::owned_arm> arm,
                                         praxis::manipulator::screw_chain chain)
        : imgui_window(std::move(name))
        , m_twist(Eigen::Vector<float, 6>::Zero())
        , m_seen(std::move(seen))
        , m_chain(std::move(chain))
        , m_arm(std::move(arm))
        , m_reported(std::make_shared<standing_refusal>())
        , m_frame(jog_frame::world_at_tool, {jog_frame::world_at_tool, jog_frame::space, jog_frame::body}, frame_labels)
{
    spdlog::info(std::format("The Jacobian jog window opened over {} joints", m_chain.joint_count()));
}

void jacobian_jog_window::render()
{
    const std::shared_ptr<const praxis::manipulator::arm_snapshot> seen = m_seen.read();

    ImGui::Begin(display_name().c_str());
    praxis::manipulator::render_option_cycle("Frame", m_frame);
    const bool held = render_twist_sliders(m_twist);
    if(!held || m_twist.isZero(0.f))
        m_reported->store(std::nullopt);
    else
        jog(praxis::scheduler::step_delta{praxis::scheduler::seconds(ImGui::GetIO().DeltaTime)});

    render_outcome(seen.get());
    ImGui::End();
}

void jacobian_jog_window::render_outcome(const praxis::manipulator::arm_snapshot *seen) const
{
    const std::optional<std::string> said = outcome_of(seen, m_frame.value());
    if(said)
        ImGui::TextWrapped("%s", said->c_str());
}

void jacobian_jog_window::jog(praxis::scheduler::step_delta elapsed) const
{
    const praxis::twist asked                        = m_twist.cast<double>();
    const jog_frame frame                            = m_frame.value();
    const praxis::manipulator::joint_limits within   = m_chain.limits;
    const std::shared_ptr<standing_refusal> reported = m_reported;

    praxis::manipulator::command(m_arm, [asked, frame, within, elapsed, reported](praxis::manipulator::robot_controller &control, praxis::manipulator::scene_robot &driven)
                                 { step_once(control, driven, frame, asked, within, elapsed, *reported); });
}

}

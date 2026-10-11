#ifndef HPP_GUARD_AIS4104_APPLICATION_JACOBIAN_JOG_WINDOW_H
#define HPP_GUARD_AIS4104_APPLICATION_JACOBIAN_JOG_WINDOW_H

#include "jacobian_displacement.h"

#include <praxis/manipulator/arm_state.h>
#include <praxis/manipulator/screw_chain.h>
#include <praxis/manipulator/arm_snapshot.h>
#include <praxis/manipulator/option_cycle.h>

#include <praxis/scene/imgui_window.h>

#include <praxis/extension/refusal.h>

#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <string>
#include <optional>

namespace ais4104 {

// Answers whether one of the six twist sliders is held this frame. The reset button is the last
// widget drawn, so the item rectangle standing after the call is the reset button's.
bool render_twist_sliders(Eigen::Vector<float, 6> &twist);

class jacobian_jog_window : public praxis::scene::imgui_window
{
    // The step runs on the arm's own strand and the panel is drawn on the renderer's, so the
    // refusal a report is throttled against is shared between the two rather than held by either.
    using standing_refusal = std::atomic<std::optional<praxis::refusal>>;

public:
    jacobian_jog_window(std::string name, praxis::manipulator::arm_reader seen, std::weak_ptr<praxis::manipulator::owned_arm> arm, praxis::manipulator::screw_chain chain);

    void render() override;

private:
    Eigen::Vector<float, 6> m_twist;
    praxis::manipulator::arm_reader m_seen;
    praxis::manipulator::screw_chain m_chain;
    std::weak_ptr<praxis::manipulator::owned_arm> m_arm;
    std::shared_ptr<standing_refusal> m_reported;
    praxis::manipulator::option_cycle<jog_frame, 3> m_frame;

    void render_outcome(const praxis::manipulator::arm_snapshot *seen) const;

    void jog(praxis::scheduler::step_delta elapsed) const;
};

}

#endif

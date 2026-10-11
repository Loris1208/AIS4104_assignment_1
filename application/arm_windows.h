#ifndef HPP_GUARD_AIS4104_APPLICATION_ARM_WINDOWS_H
#define HPP_GUARD_AIS4104_APPLICATION_ARM_WINDOWS_H

#include <praxis/presets/arm.h>
#include <praxis/presets/arm_scenarios.h>

#include <praxis/manipulator/compose_arm.h>

#include <praxis/scene/imgui_window.h>

#include <praxis/config/document.h>

#include <memory>
#include <string>

namespace ais4104 {

// Where the document sets pose_target_window/opens true, a tooling composition opens a Pose target
// window before Tool frame motion, and the two move one shared pose.
praxis::manipulator::arm_composition composing(std::string named, praxis::presets::arm_scenario_kind kind, praxis::presets::arm_scenario state, const praxis::config::document &values,
                                               praxis::manipulator::arm_composition composed);

// Where the document sets tool_window/opens false, praxis's Tool window is left out of the composed
// windows; the tool stays seated as praxis seats it at composition, and the offset the arm then
// publishes is logged once it has published one. A composition praxis declined is left empty.
praxis::manipulator::arm_composition without_tool_window(praxis::manipulator::arm_composition composed, const praxis::config::document &values);

bool is_tool_window(const std::shared_ptr<praxis::scene::imgui_window> &window);

// The window without_tool_window adds to report the seat, which draws nothing.
bool is_seat_report(const std::shared_ptr<praxis::scene::imgui_window> &window);

// Where the document sets pose_window/frames to flange or tool, a forward kinematics or tooling
// composition's Pose window reads that frame alone and draws no Frame control. A composition praxis
// declined is left empty.
praxis::manipulator::arm_composition with_offered_pose_frames(praxis::manipulator::arm_composition composed, praxis::presets::arm_scenario_kind kind,
                                                              const praxis::config::document &values);

}

#endif

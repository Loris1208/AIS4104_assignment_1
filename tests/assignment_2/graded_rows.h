#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_2_GRADED_ROWS_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_2_GRADED_ROWS_H

#include "modeling/chains.h"

#include <Eigen/Core>

#include <array>
#include <vector>
#include <utility>
#include <string_view>

namespace ais4104 {

using chain_pair = std::pair<Eigen::Matrix4d, std::vector<Eigen::Vector6d>>;

// Radians, meters, and the Frobenius distance of the home rotation block from orthonormal.
constexpr double home_rotation_radians   = 1e-6;
constexpr double home_translation_meters = 1e-6;
constexpr double home_rigidity           = 1e-6;

// Radians between two screw axes' directions, the defect of an axis's length from unit, and meters
// between their linear parts once each is divided by its own angular norm.
constexpr double screw_direction_radians = 1e-6;
constexpr double screw_length_defect     = 1e-6;
constexpr double screw_moment_meters     = 1e-6;

struct chain_row
{
    std::string_view task;
    std::string_view slot;
    std::string_view document;
    chain_pair (*derive)(const praxis::rigid_motion::screw_ops &screw);
};

constexpr std::array<chain_row, 2> chain_rows{{
        {"1a", "chains.hardcoded_kr6r900sixx_chain", "kr6r900sixx-kinematic-modeling.xml", &modeling::hardcoded_kr6r900sixx_chain},
        {"1b", "chains.hardcoded_lbr_med14_r820_chain", "lbr-med14-r820-kinematic-modeling.xml", &modeling::hardcoded_lbr_med14_r820_chain},
}};

// Radians and meters between the tool transformation a function answers and the one its document states, and the Frobenius distance of its rotation block from orthonormal.
constexpr double tool_rotation_radians   = 1e-6;
constexpr double tool_translation_meters = 1e-6;
constexpr double tool_rigidity           = 1e-6;

struct tool_row
{
    std::string_view task;
    std::string_view slot;
    std::string_view document;
    Eigen::Matrix4d (*derive)(const praxis::rigid_motion::frame_ops &frames);
};

constexpr std::array<tool_row, 2> tool_rows{{
        {"3a", "chains.hardcoded_kr6r900sixx_curved_rod_tool_offset", "kr6r900sixx-curved-rod.xml", &modeling::hardcoded_kr6r900sixx_curved_rod_tool_offset},
        {"3b", "chains.hardcoded_kr6r900sixx_humerus_tool_offset", "kr6r900sixx-biomechanics.xml", &modeling::hardcoded_kr6r900sixx_humerus_tool_offset},
}};

}

#endif

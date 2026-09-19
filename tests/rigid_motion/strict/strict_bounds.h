#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_STRICT_BOUNDS_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_STRICT_BOUNDS_H

#include <praxis/evaluation/residual.h>

#include <catch2/catch_test_macros.hpp>

#include <format>
#include <string>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ais4104 {

// Where a bound stands against the tolerance the platform publishes for its slot's kind. The suite
// requires the comparison to come out the way the standing names, in both directions, so a bound
// that stops being tighter than the platform's is a red row rather than a quiet pass.
enum class bound_standing : std::uint8_t
{
    tighter,
    at_the_platform,
    above_the_platform,
};

// The two halves are in the units the slot's residual kind states. A half whose kind carries no
// verdict for it holds the platform's own number, so no row is held to a measurement of nothing.
struct strict_bound
{
    double magnitude;
    double linear_metres;
    bound_standing magnitude_standing;
    bound_standing linear_standing;
};

// How many slots the measurement grades strictly. The suite's own runner table is sized by this, so
// a bound without a section, or a section without a bound, does not build.
constexpr std::size_t strictly_graded_slots = 12;

constexpr strict_bound rotation_matrix_from_euler_bound{1.1102230246259896e-15, 1e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound rotation_matrix_from_axis_angle_bound{1.0962573991985805e-15, 1e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound adjoint_matrix_from_rotation_position_bound{2.6645352591003757e-15, 1e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound adjoint_matrix_from_transform_bound{2.6645352591003757e-15, 1e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound adjoint_map_bound{5.3290705182007514e-15, 9.9999999999999998e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound screw_axis_from_angular_linear_bound{1.2392740186667945e-14, 9.9999999999999998e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound matrix_exponential_so3_bound{1.0955764188730678e-15, 1e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound matrix_exponential_se3_bound{4.7533651739390202e-15, 6.2842938400131246e-15, bound_standing::tighter, bound_standing::tighter};
constexpr strict_bound matrix_exponential_screw_bound{6.8794645199604165e-15, 9.2302208946411071e-15, bound_standing::tighter, bound_standing::tighter};
constexpr strict_bound matrix_logarithm_so3_bound{3.638217643061203e-15, 9.9999999999999998e-13, bound_standing::tighter, bound_standing::at_the_platform};
constexpr strict_bound matrix_logarithm_se3_rp_bound{7.0072911571827325e-15, 1.8887887874647666e-12, bound_standing::tighter, bound_standing::above_the_platform};
constexpr strict_bound matrix_logarithm_se3_bound{6.704610857842959e-15, 7.4076173489607477e-13, bound_standing::tighter, bound_standing::tighter};

constexpr praxis::evaluation::tolerance_pair allowed_of(const strict_bound &bound)
{
    return praxis::evaluation::tolerance_pair{bound.magnitude, bound.linear_metres};
}

constexpr bound_standing standing_of(double bound, double published)
{
    if(bound < published)
        return bound_standing::tighter;
    if(published < bound)
        return bound_standing::above_the_platform;

    return bound_standing::at_the_platform;
}

inline std::string standing_line(std::string_view half, double bound, double published)
{
    return std::format("the {} bound {:.17e} against the published {:.17e}", half, bound, published);
}

// Both halves, both directions: strictly under where the standing says tighter, equal where it says
// at the platform, strictly over where it says above it. The published number is the slot's own,
// which a table row may set instead of taking its kind's default.
inline void standing_matches(const strict_bound &bound, const praxis::evaluation::tolerance_pair &published)
{
    INFO(standing_line("magnitude", bound.magnitude, published.magnitude));
    REQUIRE(standing_of(bound.magnitude, published.magnitude) == bound.magnitude_standing);

    INFO(standing_line("linear", bound.linear_metres, published.linear_metres));
    REQUIRE(standing_of(bound.linear_metres, published.linear_metres) == bound.linear_standing);
}

}

#endif

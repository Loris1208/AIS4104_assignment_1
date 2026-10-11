#ifndef HPP_GUARD_AIS4104_APPLICATION_JACOBIAN_DISPLACEMENT_H
#define HPP_GUARD_AIS4104_APPLICATION_JACOBIAN_DISPLACEMENT_H

#include <praxis/manipulator/types.h>

#include <praxis/rigid_motion/types.h>

#include <praxis/extension/refusal.h>

#include <praxis/scheduler/task.h>

#include <praxis/compat/expected.h>

#include <Eigen/Core>

#include <cstdint>

namespace ais4104 {

// The extent a twist slider is drawn to, in both directions from rest.
extern const float twist_slider_bound;

// Which frame a twist the jog asks for is expressed in. The world-at-tool frame translates the tool
// centre point along the world axes and rotates about world axes through that point; the other two
// are the frames the two Jacobians the arm publishes are expressed in.
enum class jog_frame : std::uint8_t
{
    world_at_tool,
    space,
    body
};

// Re-references a Jacobian's twists to a frame holding the world axes at `point`: the adjoint of
// the transform carrying the world origin to that point, applied on the left, so the angular rows
// are unchanged and the linear rows answer the velocity of that point rather than of the
// space-frame origin. Lynch & Park, Modern Robotics, section 3.3.2.
praxis::manipulator::jacobian referenced_to(const praxis::manipulator::jacobian &of, const Eigen::Vector3d &point);

// One tick of the jog: `at` displaced by the least-squares joint rates for `asked` over `elapsed`,
// the time since the previous tick, and clamped into `within`. A stalled frame's interval is
// integrated no further than the longest one tick admits, and an interval that is not positive
// moves nothing. The rates are damped only as the chain approaches a singularity, and are scaled as
// a whole to hold every joint inside the velocity limit `within` declares. A Jacobian whose column
// count disagrees with the joint count, and a chain whose velocity limits are absent, mis-sized,
// non-finite or not strictly positive, are refused as unsupported input rather than added to the
// configuration.
praxis::expected<praxis::manipulator::joint_vector, praxis::refusal> jogged_configuration(const praxis::manipulator::jacobian &of, const praxis::twist &asked,
                                                                                          const praxis::manipulator::joint_vector &at, const praxis::manipulator::joint_limits &within,
                                                                                          praxis::scheduler::step_delta elapsed);

}

#endif

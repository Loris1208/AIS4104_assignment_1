#include "trajectory_generation/via_points.h"
#include "trajectory_generation/via_point_trajectory.h"

#include <span>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace ais4104::trajectory {

namespace {

class via_point_trajectory final : public praxis::trajectory::trajectory_generator
{
public:
    via_point_trajectory(std::vector<double> knots, std::vector<cubic_segment> segments)
            : m_knots(std::move(knots))
            , m_segments(std::move(segments))
    {
    }

    praxis::expected<praxis::trajectory::trajectory_sample, praxis::refusal> sample(double t) const override
    {
        const double held         = std::max(m_knots.front(), std::min(t, m_knots.back()));
        const std::size_t segment = segment_at(held);

        return segment_sample(m_segments[segment], held - m_knots[segment]);
    }

    double duration() const override
    {
        return m_knots.back();
    }

private:
    std::vector<double> m_knots;
    std::vector<cubic_segment> m_segments;

    // A time at an interior knot belongs to the segment it begins, and the last knot to the last segment.
    std::size_t segment_at(double held) const
    {
        const auto after = std::upper_bound(m_knots.begin(), m_knots.end(), held);

        return std::min(static_cast<std::size_t>(after - m_knots.begin()), m_segments.size()) - 1u;
    }
};

bool enough(std::span<const praxis::trajectory::via_point> via_points)
{
    return via_points.size() >= 2u;
}

bool starts_at_zero(std::span<const praxis::trajectory::via_point> via_points)
{
    return via_points.front().time == 0.0;
}

bool times_advance(std::span<const praxis::trajectory::via_point> via_points)
{
    for(std::size_t k = 1u; k < via_points.size(); ++k)
        if(!std::isfinite(via_points[k].time) || !(via_points[k].time > via_points[k - 1u].time))
            return false;

    return true;
}

bool widths_agree(std::span<const praxis::trajectory::via_point> via_points)
{
    const Eigen::Index width = via_points.front().position.size();

    return std::ranges::all_of(via_points, [width](const praxis::trajectory::via_point &at) { return at.position.size() == width && at.velocity.size() == width; });
}

bool all_finite(std::span<const praxis::trajectory::via_point> via_points)
{
    return std::ranges::all_of(via_points, [](const praxis::trajectory::via_point &at) { return at.position.allFinite() && at.velocity.allFinite(); });
}

std::vector<double> knots_of(std::span<const praxis::trajectory::via_point> via_points)
{
    std::vector<double> knots;
    knots.reserve(via_points.size());
    for(const praxis::trajectory::via_point &at : via_points)
        knots.push_back(at.time);

    return knots;
}

praxis::expected<std::vector<cubic_segment>, praxis::refusal> segments_through(std::span<const praxis::trajectory::via_point> via_points)
{
    std::vector<cubic_segment> segments;
    segments.reserve(via_points.size() - 1u);
    for(std::size_t j = 0u; j + 1u < via_points.size(); ++j)
    {
        praxis::expected<cubic_segment, praxis::refusal> segment = segment_coefficients(via_points[j], via_points[j + 1u]);
        if(!segment)
            return praxis::unexpected(segment.error());

        segments.push_back(std::move(*segment));
    }

    return segments;
}

}

praxis::expected<std::unique_ptr<praxis::trajectory::trajectory_generator>, praxis::refusal> joint_space_via_points(std::span<const praxis::trajectory::via_point> via_points)
{
    if(!enough(via_points) || !starts_at_zero(via_points) || !times_advance(via_points) || !widths_agree(via_points) || !all_finite(via_points))
        return praxis::unexpected(praxis::refusal::unsupported_input);

    praxis::expected<std::vector<cubic_segment>, praxis::refusal> segments = segments_through(via_points);
    if(!segments)
        return praxis::unexpected(segments.error());

    return std::make_unique<via_point_trajectory>(knots_of(via_points), std::move(*segments));
}

}

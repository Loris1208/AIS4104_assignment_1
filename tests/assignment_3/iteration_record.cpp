#include "iteration_record.h"
#include "inverse_kinematics_cases.h"

#include <praxis/evaluation/residual.h>
#include <praxis/evaluation/comparators.h>

#include <praxis/rigid_motion/screw.h>
#include <praxis/rigid_motion/types.h>

#include <span>
#include <cmath>
#include <format>
#include <string>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

namespace ais4104 {
namespace {

using entry   = praxis::manipulator::iteration_state;
using finding = std::optional<std::string>;

struct own_errors
{
    double angular;
    double linear_body;
    double linear_space;
};

bool agrees(double recorded, double recomputed, double absolute, double relative)
{
    return std::abs(recorded - recomputed) <= absolute + relative * std::abs(recomputed);
}

bool agrees_on_error(double recorded, double recomputed)
{
    return agrees(recorded, recomputed, record_error_absolute, record_error_relative);
}

answer<Eigen::Matrix4d> pose_at(const praxis::manipulator::screw_chain &chain, const Eigen::VectorXd &theta)
{
    return reference_manipulator().fk.forward_kinematics(reference_motions().screw, chain.home, chain.space_screws, theta);
}

answer<own_errors> errors_at(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &theta)
{
    const praxis::rigid_motion::screw_ops &screw = reference_motions().screw;
    const answer<Eigen::Matrix4d> reached        = pose_at(chain, theta);
    if(!reached)
        return praxis::unexpected(reached.error());

    const Eigen::Matrix4d body_error                              = reference_motions().frame.inverse(*reached) * desired;
    const answer<std::pair<praxis::screw_axis, double>> logarithm = screw.matrix_logarithm_se3(body_error);
    if(!logarithm)
        return praxis::unexpected(logarithm.error());

    const praxis::twist body          = logarithm->first * logarithm->second;
    const answer<praxis::twist> space = screw.adjoint_map(body, *reached);
    if(!space)
        return praxis::unexpected(space.error());

    return own_errors{body.head<3>().norm(), body.tail<3>().norm(), space->tail<3>().norm()};
}

finding unreadable_entry(std::span<const entry> record, std::size_t joints)
{
    for(std::size_t k = 0; k < record.size(); ++k)
    {
        const entry &each = record[k];
        if(static_cast<std::size_t>(each.joint_positions.size()) != joints)
            return std::format(" record entry {}: holds {} joints for a chain of {}", k, each.joint_positions.size(), joints);
        if(!each.joint_positions.allFinite() || !std::isfinite(each.angular_error) || !std::isfinite(each.linear_error) || !std::isfinite(each.step_norm))
            return std::format(" record entry {}: a field is not finite", k);
    }

    return std::nullopt;
}

finding broken_index(std::span<const entry> record)
{
    if(record.empty())
        return std::nullopt;
    if(record.front().index > 1u)
        return std::format(" record entry 0: index {} is neither 0 nor 1", record.front().index);

    for(std::size_t k = 1; k < record.size(); ++k)
        if(record[k].index != record.front().index + k)
            return std::format(" record entry {}: index {} does not follow {}", k, record[k].index, record[k - 1].index);

    return std::nullopt;
}

finding wrong_step(std::span<const entry> record, const Eigen::VectorXd &seed)
{
    for(std::size_t k = 0; k < record.size(); ++k)
    {
        const Eigen::VectorXd &before = k == 0 ? seed : record[k - 1].joint_positions;
        const double distance         = (record[k].joint_positions - before).norm();
        if(!agrees(record[k].step_norm, distance, record_step_absolute, record_step_relative))
            return std::format(" record entry {}: step norm {:.6e} is not the distance from the configuration before it ({:.6e}, off by {:.1e})", k, record[k].step_norm, distance,
                               std::abs(record[k].step_norm - distance));
    }

    return std::nullopt;
}

finding error_mismatch(std::size_t k, const entry &each, const own_errors &own)
{
    if(!agrees_on_error(each.angular_error, own.angular))
        return std::format(" record entry {}: angular error {:.6e} is not the error at its own configuration ({:.6e}, off by {:.1e})", k, each.angular_error, own.angular,
                           std::abs(each.angular_error - own.angular));
    if(!agrees_on_error(each.linear_error, own.linear_body) && !agrees_on_error(each.linear_error, own.linear_space))
        return std::format(" record entry {}: linear error {:.6e} is not the error at its own configuration ({:.6e} body, {:.6e} space, off by {:.1e})", k, each.linear_error,
                           own.linear_body, own.linear_space, std::min(std::abs(each.linear_error - own.linear_body), std::abs(each.linear_error - own.linear_space)));

    return std::nullopt;
}

finding wrong_error(std::span<const entry> record, const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired)
{
    for(std::size_t k = 0; k < record.size(); ++k)
    {
        const answer<own_errors> own = errors_at(chain, desired, record[k].joint_positions);
        if(!own)
            return std::format(" record entry {}: the reference read no error at its configuration ({})", k, refusal_name(own.error()));

        finding found = error_mismatch(k, record[k], *own);
        if(found)
            return found;
    }

    return std::nullopt;
}

finding entry_finding(std::span<const entry> record, const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed)
{
    finding found = unreadable_entry(record, chain.joint_count());
    if(!found)
        found = broken_index(record);
    if(!found)
        found = wrong_step(record, seed);
    if(!found)
        found = wrong_error(record, chain, desired);

    return found;
}

finding wrong_end(std::span<const entry> record, const praxis::manipulator::screw_chain &chain, const Eigen::VectorXd &seed, const Eigen::VectorXd &answered)
{
    const answer<Eigen::Matrix4d> ended   = pose_at(chain, record.empty() ? seed : record.back().joint_positions);
    const answer<Eigen::Matrix4d> reached = pose_at(chain, answered);
    if(!ended || !reached)
        return std::format(" record end: the reference forward kinematics refused ({})", refusal_name(!ended ? ended.error() : reached.error()));

    const praxis::evaluation::residual seen = praxis::evaluation::pose_residual(*reached, *ended);
    if(seen.magnitude <= record_end_radians && seen.linear_error_metres <= record_end_metres)
        return std::nullopt;
    if(record.empty())
        return std::format(" record empty, but the answer is not at the seed's pose (residual {:.6e} / {:.6e})", seen.magnitude, seen.linear_error_metres);

    return std::format(" record ends at a pose the answer is not at (residual {:.6e} / {:.6e})", seen.magnitude, seen.linear_error_metres);
}

}

record_reading record_of(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed, const answer<void> &solved,
                         const praxis::manipulator::ik_result &result)
{
    if(!solved || result.solutions.empty())
        return record_reading{true, " record not read (no answer)"};

    const std::span<const entry> record = result.iterations;
    finding found                       = entry_finding(record, chain, desired, seed);
    if(!found)
        found = wrong_end(record, chain, seed, result.solutions.front());

    return found ? record_reading{false, *found} : record_reading{true, " record consistent"};
}

record_reading kept_record_of(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed, const praxis::manipulator::ik_result &result)
{
    const std::span<const entry> record = result.iterations;
    finding found                       = entry_finding(record, chain, desired, seed);
    if(!found && record.empty())
        found = " record empty";
    if(!found && !(record.front().joint_positions.array() == seed.array()).all())
        found = " record entry 0: the configuration is not the seed";

    return found ? record_reading{false, *found} : record_reading{true, " record consistent"};
}

}

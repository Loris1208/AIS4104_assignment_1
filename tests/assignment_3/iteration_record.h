#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_3_ITERATION_RECORD_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_3_ITERATION_RECORD_H

#include "suite_rows.h"

#include <praxis/manipulator/kinematics.h>
#include <praxis/manipulator/screw_chain.h>

#include <Eigen/Core>

#include <string>

namespace ais4104 {

struct record_reading
{
    bool consistent;
    std::string clause;
};

// Modern Robotics, section 6.2.2 and the table of Example 6.1: each entry pairs a configuration with the
// twist read at that configuration. The record's length is never read.
record_reading record_of(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed, const answer<void> &solved,
                         const praxis::manipulator::ik_result &result);

// The record a refused solve kept: non-empty, entry 0 the seed, and every entry read as record_of reads it; its end is not read.
record_reading kept_record_of(const praxis::manipulator::screw_chain &chain, const Eigen::Matrix4d &desired, const Eigen::VectorXd &seed, const praxis::manipulator::ik_result &result);

}

#endif

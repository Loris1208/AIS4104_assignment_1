#ifndef HPP_GUARD_AIS4104_APPLICATION_ARM_SLOT_REPORT_H
#define HPP_GUARD_AIS4104_APPLICATION_ARM_SLOT_REPORT_H

#include "slot_report.h"

#include <praxis/manipulator/slots.h>
#include <praxis/manipulator/capabilities.h>

#include <praxis/trajectory/slots.h>
#include <praxis/trajectory/capabilities.h>

#include <praxis/optimization/slots.h>
#include <praxis/optimization/capabilities.h>

#include <praxis/extension/slot_set.h>

#include <vector>
#include <cstddef>

namespace ais4104 {

struct owned_slots
{
    praxis::manipulator::forward_kinematics_slot_set fk;
    praxis::manipulator::differential_kinematics_slot_set dk;
    praxis::manipulator::inverse_kinematics_slot_set ik;
    praxis::manipulator::robot_slot_set robot;
    praxis::trajectory::time_scaling_slot_set time_scaling;
    praxis::trajectory::path_slot_set path;
    praxis::trajectory::trajectory_slot_set trajectory;
    praxis::optimization::solver_slot_set solver;
    praxis::optimization::minimizer_slot_set minimizer;
};

template<praxis::slot_enumeration Slot>
constexpr praxis::basic_slot_set<Slot> every_slot()
{
    return ~praxis::basic_slot_set<Slot>{};
}

owned_slots offered_owned_slots();

std::vector<classified_slot> classify_arm_slots(const praxis::manipulator::capabilities &arm, const praxis::trajectory::capabilities &shapes,
                                                const praxis::optimization::capabilities &solving, const owned_slots &owned, std::size_t cases);

void report_arm_slot_state(const praxis::manipulator::capabilities &arm, const praxis::trajectory::capabilities &shapes, const praxis::optimization::capabilities &solving,
                           const owned_slots &owned);

}

#endif

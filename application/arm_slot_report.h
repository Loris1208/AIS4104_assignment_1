#ifndef HPP_GUARD_AIS4104_APPLICATION_ARM_SLOT_REPORT_H
#define HPP_GUARD_AIS4104_APPLICATION_ARM_SLOT_REPORT_H

#include "slot_report.h"

#include <praxis/manipulator/capabilities.h>

#include <vector>
#include <cstddef>

namespace ais4104 {

std::vector<classified_slot> classify_arm_slots(const praxis::manipulator::capabilities &arm, std::size_t cases);

void report_arm_slot_state(const praxis::manipulator::capabilities &arm);

}

#endif

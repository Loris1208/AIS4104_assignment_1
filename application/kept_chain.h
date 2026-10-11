#ifndef HPP_GUARD_AIS4104_APPLICATION_KEPT_CHAIN_H
#define HPP_GUARD_AIS4104_APPLICATION_KEPT_CHAIN_H

#include <praxis/presets/arm.h>
#include <praxis/presets/routes.h>

#include <praxis/manipulator/compose_arm.h>

#include <praxis/config/binding.h>
#include <praxis/config/document.h>

#include <optional>

namespace ais4104 {

struct opened_arm
{
    praxis::manipulator::arm_composition windows;
    praxis::config::binding announced;
    praxis::config::document carried;
    praxis::presets::arm_scenario opening;
};

// An arm document naming a chain under screw_table/document opens over the copy written answers for
// that name: the Chain window reads and saves the chain there, and the View and Tool windows open at
// what the copy keeps, each value it does not keep at the arm document's. opening is the scenario
// those windows open at; an arm naming none answers nothing.
std::optional<opened_arm> opened_keeping(const praxis::presets::arm_scenario &machine, const praxis::config::document &arm, const praxis::presets::document_route &written);

}

#endif

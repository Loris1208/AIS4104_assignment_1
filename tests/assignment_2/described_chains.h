#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_2_DESCRIBED_CHAINS_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_2_DESCRIBED_CHAINS_H

#include <praxis/presets/arm.h>

#include <praxis/manipulator/screw_chain.h>

#include <praxis/compat/expected.h>

#include <string>
#include <string_view>

namespace ais4104 {

// The scenario a document in application/documents describes, read through this application's own
// keyspace, with its description resolved against the resources deployed beside the test.
praxis::expected<praxis::presets::arm_scenario, std::string> described_scenario(std::string_view document);

// praxis's own derivation of the chain that scenario's description carries. Each document's
// description is loaded once per run.
praxis::expected<praxis::manipulator::screw_chain, std::string> described_chain(std::string_view document);

}

#endif

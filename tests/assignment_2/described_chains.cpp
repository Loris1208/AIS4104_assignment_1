#include "described_chains.h"

#include "arm_keyspace.h"

#include <praxis/presets/arm.h>
#include <praxis/presets/arm_registration.h>

#include <praxis/manipulator/screw_chain.h>
#include <praxis/manipulator/capabilities.h>

#include <praxis/config/store.h>
#include <praxis/config/binding.h>

#include <praxis/compat/expected.h>

#include <meios/urdf/load.h>

#include <map>
#include <array>
#include <format>
#include <string>
#include <filesystem>
#include <functional>
#include <string_view>

#ifndef AIS4104_DOCUMENT_DIRECTORY
    #define AIS4104_DOCUMENT_DIRECTORY "application/documents"
#endif

#ifndef AIS4104_DEPLOYED_DIRECTORY
    #define AIS4104_DEPLOYED_DIRECTORY "."
#endif

namespace ais4104 {
namespace {

using derived_chain = praxis::expected<praxis::manipulator::screw_chain, std::string>;

derived_chain chain_derived_from(std::string_view document)
{
    const praxis::expected<praxis::presets::arm_scenario, std::string> scenario = described_scenario(document);
    if(!scenario)
        return praxis::unexpected(scenario.error());

    const auto loaded = meios::load(scenario->description, scenario->options);
    if(!loaded)
        return praxis::unexpected(std::format("The description {} names does not load: {}", document, loaded.error().message));

    const auto derived = praxis::manipulator::baseline().modeling.build_chain(loaded->robot);
    if(!derived)
        return praxis::unexpected(std::format("praxis derives no chain from the description {} names", document));

    return derived.value();
}

}

praxis::expected<praxis::presets::arm_scenario, std::string> described_scenario(std::string_view document)
{
    const std::filesystem::path documents(AIS4104_DOCUMENT_DIRECTORY);
    const std::filesystem::path deployed(AIS4104_DEPLOYED_DIRECTORY);
    const praxis::config::binding bound{assignment_arm_keyspace(), praxis::config::resolve(documents / document, documents), praxis::config::expectation::partial};

    const praxis::config::outcome read = praxis::config::load_or_defaults(bound);
    if(read.failure)
        return praxis::unexpected(std::format("The document {} does not read: {}", document, read.failure->message));

    const std::array<std::filesystem::path, 2> roots{deployed / "urdf", deployed};

    return praxis::presets::read_arm(read.values, roots);
}

praxis::expected<praxis::manipulator::screw_chain, std::string> described_chain(std::string_view document)
{
    static std::map<std::string, derived_chain, std::less<>> derived;

    auto found = derived.find(document);
    if(found == derived.end())
        found = derived.emplace(std::string(document), chain_derived_from(document)).first;

    return found->second;
}

}

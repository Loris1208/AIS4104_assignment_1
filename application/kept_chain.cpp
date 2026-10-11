#include "kept_chain.h"

#include <praxis/presets/screw_table.h>
#include <praxis/presets/kept_modeling.h>

#include <string>
#include <utility>
#include <optional>

namespace ais4104 {

std::optional<opened_arm> opened_keeping(const praxis::presets::arm_scenario &machine, const praxis::config::document &arm, const praxis::presets::document_route &written)
{
    const std::string named = arm.text("screw_table/document").value_or(std::string());
    if(named.empty())
        return std::nullopt;

    const praxis::config::binding keeping{praxis::presets::screw_table_keyspace(), written(named), praxis::config::expectation::partial};
    praxis::presets::kept_modeling kept = praxis::presets::arm_windows_kept_modeling(machine, keeping);

    return opened_arm{std::move(kept.composed), std::move(kept.bound), std::move(kept.carried), std::move(kept.opening)};
}

}

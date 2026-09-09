#include "offered.h"
#include "configuration.h"

#include <praxis/presets/arrangements.h>

#include <praxis/config/store.h>

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace ais4104 {

std::vector<std::string> register_offered(const std::shared_ptr<praxis::scene::preset_registry> &registry, const document_store &mine, const praxis::config::document &values,
                                          const std::shared_ptr<write_back> &writing, const praxis::rigid_motion::capabilities &motions)
{
    const std::vector<praxis::config::location> reading = preset_locations(values, mine);

    return praxis::presets::register_arrangements(
            registry, reading, [mine](const std::filesystem::path &named) { return mine.reading(named); },
            [writing](const praxis::config::binding &at, const praxis::config::document &carried) { writing->composing(at, carried); }, motions);
}

}

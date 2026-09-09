#include "document_store.h"

#include <spdlog/spdlog.h>

#include <format>
#include <utility>
#include <filesystem>
#include <system_error>

namespace ais4104 {

namespace {

void reproduce(const std::filesystem::path &seed, const std::filesystem::path &target)
{
    std::error_code failed;
    std::filesystem::copy_file(seed, target, std::filesystem::copy_options::skip_existing, failed);
    if(failed)
        spdlog::error(std::format("The document {} was not copied to {}: {}", seed.string(), target.string(), failed.message()));
}

}

document_store::document_store(std::filesystem::path seeds, std::filesystem::path state)
        : m_seeds(std::move(seeds))
        , m_state(std::move(state))
{
}

const std::filesystem::path &document_store::seeds() const
{
    return m_seeds;
}

praxis::config::location document_store::reading(const std::filesystem::path &named) const
{
    const praxis::config::location copy = praxis::config::resolve(named, m_state);

    return std::filesystem::exists(copy.resolved) ? copy : praxis::config::resolve(named, m_seeds);
}

praxis::config::location document_store::writing(const std::filesystem::path &named) const
{
    const praxis::config::location target = praxis::config::resolve(named, m_state);
    const std::filesystem::path directory = target.resolved.parent_path();

    std::error_code failed;
    std::filesystem::create_directories(directory, failed);
    if(failed)
        spdlog::error(std::format("The directory {} could not be created: {}", directory.string(), failed.message()));

    const praxis::config::location seed = praxis::config::resolve(named, m_seeds);
    if(!std::filesystem::exists(target.resolved) && std::filesystem::exists(seed.resolved))
        reproduce(seed.resolved, target.resolved);

    return target;
}

}

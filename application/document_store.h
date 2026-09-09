#ifndef HPP_GUARD_AIS4104_APPLICATION_DOCUMENT_STORE_H
#define HPP_GUARD_AIS4104_APPLICATION_DOCUMENT_STORE_H

#include <praxis/config/store.h>

#include <filesystem>

namespace ais4104 {

class document_store
{
public:
    document_store(std::filesystem::path seeds, std::filesystem::path state);

    const std::filesystem::path &seeds() const;

    praxis::config::location reading(const std::filesystem::path &named) const;

    praxis::config::location writing(const std::filesystem::path &named) const;

private:
    std::filesystem::path m_seeds;
    std::filesystem::path m_state;
};

}

#endif

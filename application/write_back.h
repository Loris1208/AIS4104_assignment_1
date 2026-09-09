#ifndef HPP_GUARD_AIS4104_APPLICATION_WRITE_BACK_H
#define HPP_GUARD_AIS4104_APPLICATION_WRITE_BACK_H

#include "document_store.h"

#include <praxis/scene/visualizer.h>

#include <praxis/config/writer.h>
#include <praxis/config/binding.h>
#include <praxis/config/document.h>
#include <praxis/config/declaration.h>
#include <praxis/config/configurable.h>

#include <span>
#include <memory>
#include <vector>
#include <cstdint>

namespace ais4104 {

enum class leaving_choice : std::uint8_t
{
    ask,
    keep,
    discard
};

void declare_leaving(praxis::config::declaration &shape);

class write_back
{
public:
    write_back(praxis::config::binding bound, praxis::config::document carried, praxis::config::binding preferences, const praxis::config::document &preferred, document_store mine);

    void composing(praxis::config::binding bound, praxis::config::document carried);

    bool anything_to_decide(std::span<const praxis::config::configurable *const> shown);

    void resolve(praxis::scene::leaving_answer chosen, std::span<const praxis::config::configurable *const> shown);

    void save(std::span<const praxis::config::configurable *const> shown);

private:
    praxis::config::binding m_bound;
    praxis::config::binding m_preferences;
    praxis::config::document m_carried;
    leaving_choice m_remembered;
    document_store m_mine;

    void write(std::vector<praxis::config::edit> changes);

    void remember(leaving_choice chosen);
};

void install_write_back(praxis::scene::visualizer &view, const std::shared_ptr<write_back> &through);

}

#endif

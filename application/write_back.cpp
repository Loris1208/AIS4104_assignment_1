#include "write_back.h"

#include <praxis/config/error.h>
#include <praxis/config/store.h>

#include <spdlog/spdlog.h>

#include <span>
#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace ais4104 {

namespace {

constexpr const char *leaving_key = "editing/on_leaving";

constexpr std::array<const char *, 3> leaving_choices{"ask", "keep", "discard"};

leaving_choice remembered_leaving(const praxis::config::document &values)
{
    const praxis::expected<std::string, praxis::config::error> read = values.text(leaving_key);
    for(std::size_t option = 0u; read && option < leaving_choices.size(); ++option)
        if(read.value() == leaving_choices[option])
            return static_cast<leaving_choice>(option);

    return leaving_choice::ask;
}

praxis::config::edit leaving_edit(leaving_choice chosen)
{
    return praxis::config::edit{leaving_key, leaving_choices[static_cast<std::size_t>(chosen)]};
}

bool written_into(const praxis::config::binding &into, std::span<const praxis::config::edit> changes)
{
    const praxis::expected<void, praxis::config::error> written = praxis::config::save(into, changes);
    if(!written)
        spdlog::error(std::format("The values were not written: {}", written.error().message));

    return written.has_value();
}

}

void declare_leaving(praxis::config::declaration &shape)
{
    shape.group("editing");
    shape.choice(leaving_key, std::vector<std::string>(leaving_choices.begin(), leaving_choices.end()), leaving_choices[0]);
}

write_back::write_back(praxis::config::binding bound, praxis::config::document carried, praxis::config::binding preferences, const praxis::config::document &preferred,
                       document_store mine)
        : m_bound(std::move(bound))
        , m_preferences(std::move(preferences))
        , m_carried(std::move(carried))
        , m_remembered(remembered_leaving(preferred))
        , m_mine(std::move(mine))
{
}

void write_back::composing(praxis::config::binding bound, praxis::config::document carried)
{
    m_bound   = std::move(bound);
    m_carried = std::move(carried);
}

bool write_back::anything_to_decide(std::span<const praxis::config::configurable *const> shown)
{
    const bool moved = praxis::config::anything_unsaved(shown, m_carried);
    if(m_remembered == leaving_choice::ask)
        return moved;

    if(moved && m_remembered == leaving_choice::keep)
        save(shown);

    return false;
}

void write_back::resolve(praxis::scene::leaving_answer chosen, std::span<const praxis::config::configurable *const> shown)
{
    if(chosen.keep)
        write(praxis::config::shown_edits(shown, m_carried));

    if(chosen.remember)
        remember(chosen.keep ? leaving_choice::keep : leaving_choice::discard);
}

void write_back::save(std::span<const praxis::config::configurable *const> shown)
{
    write(praxis::config::shown_edits(shown, m_carried));
}

void write_back::write(std::vector<praxis::config::edit> changes)
{
    if(changes.empty())
    {
        spdlog::info(std::format("Nothing the composition shows is unsaved, so the configuration at {} was left as it is", m_bound.at.resolved.string()));
        return;
    }

    const praxis::config::binding into{m_bound.shape, m_mine.writing(m_bound.at.given), m_bound.carries};
    if(!written_into(into, changes))
        return;

    praxis::expected<praxis::config::document, praxis::config::error> reread = praxis::config::load(into.shape, into.at);
    if(reread)
        m_carried = std::move(reread).value();
}

void write_back::remember(leaving_choice chosen)
{
    const std::array<praxis::config::edit, 1> changes{leaving_edit(chosen)};
    if(written_into(m_preferences, changes))
        m_remembered = chosen;
}

void install_write_back(praxis::scene::visualizer &view, const std::shared_ptr<write_back> &through)
{
    view.saving_through([&view, through] { through->save(view.configured()); });
    view.asking_before_release([&view, through] { return through->anything_to_decide(view.configured()); },
                               [&view, through](praxis::scene::leaving_answer chosen) { through->resolve(chosen, view.configured()); });
}

}

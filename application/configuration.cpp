#include "write_back.h"
#include "configuration.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ais4104 {

namespace {

constexpr const char *window_width  = "window/width";
constexpr const char *window_height = "window/height";
constexpr const char *window_x      = "window/x";
constexpr const char *window_y      = "window/y";
constexpr const char *level_key     = "reporting/level";

constexpr std::array<const char *, 4> reporting_levels{"debug", "info", "warning", "error"};

constexpr const char *announced_level = "info";

std::optional<int> stored(const praxis::config::document &values, const char *key)
{
    if(values.origin_of(key).kind != praxis::config::origin_kind::source)
        return std::nullopt;

    const praxis::expected<std::int64_t, praxis::config::error> read = values.integer(key);
    if(!read)
        return std::nullopt;

    return static_cast<int>(read.value());
}

std::optional<int> stored_extent(const praxis::config::document &values, const char *key)
{
    const std::optional<int> read = stored(values, key);

    return read && *read > 0 ? read : std::nullopt;
}

void append(std::vector<praxis::config::edit> &changes, const char *key, const std::optional<int> &value)
{
    if(value)
        changes.push_back(praxis::config::edit{key, std::to_string(*value)});
}

}

praxis::config::declaration preferences_keyspace()
{
    praxis::config::declaration shape("preferences");
    shape.group("window");
    shape.field(window_width, praxis::config::field_kind::integer, "0");
    shape.field(window_height, praxis::config::field_kind::integer, "0");
    shape.field(window_x, praxis::config::field_kind::integer, "0");
    shape.field(window_y, praxis::config::field_kind::integer, "0");

    shape.group("reporting");
    shape.choice(level_key, std::vector<std::string>(reporting_levels.begin(), reporting_levels.end()), announced_level);
    declare_leaving(shape);

    return shape;
}

praxis::scene::visualizer::geometry preferred_geometry(const praxis::config::document &values)
{
    praxis::scene::visualizer::geometry where;
    where.width  = stored_extent(values, window_width);
    where.height = stored_extent(values, window_height);
    where.x      = stored(values, window_x);
    where.y      = stored(values, window_y);

    return where;
}

praxis::scene::severity preferred_level(const praxis::config::document &values)
{
    const praxis::expected<std::string, praxis::config::error> read = values.text(level_key);
    for(std::size_t option = 0u; read && option < reporting_levels.size(); ++option)
        if(read.value() == reporting_levels[option])
            return static_cast<praxis::scene::severity>(option);

    return praxis::scene::reporting_level();
}

std::vector<praxis::config::edit> preferences_edits(const praxis::scene::visualizer::geometry &left, praxis::scene::severity level)
{
    std::vector<praxis::config::edit> changes;
    append(changes, window_width, left.width);
    append(changes, window_height, left.height);
    append(changes, window_x, left.x);
    append(changes, window_y, left.y);
    changes.push_back(praxis::config::edit{level_key, reporting_levels[static_cast<std::size_t>(level)]});

    return changes;
}

praxis::config::declaration application_keyspace()
{
    praxis::config::declaration shape("application");
    shape.group("presets").collection(preset_instances, "name");
    shape.field(std::string(preset_instances) + "/" + document_leaf, praxis::config::field_kind::text, "");

    return shape;
}

std::vector<std::string> preset_keys(const praxis::config::document &values)
{
    return values.identities(preset_instances);
}

praxis::expected<praxis::config::location, praxis::config::error> preset_document(const praxis::config::document &values, const std::string &key, const document_store &mine)
{
    const praxis::expected<std::string, praxis::config::error> addressed = values.key(preset_instances, key, document_leaf);
    if(!addressed)
        return praxis::unexpected(addressed.error());

    const praxis::expected<std::string, praxis::config::error> named = values.text(addressed.value());
    if(!named)
        return praxis::unexpected(named.error());

    return mine.reading(named.value());
}

std::vector<praxis::config::location> preset_locations(const praxis::config::document &values, const document_store &mine)
{
    std::vector<praxis::config::location> reading;
    for(const std::string &key : preset_keys(values))
    {
        const praxis::expected<praxis::config::location, praxis::config::error> at = preset_document(values, key, mine);
        if(at && !at.value().given.empty())
            reading.push_back(at.value());
    }

    return reading;
}

}

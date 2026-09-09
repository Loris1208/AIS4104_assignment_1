#ifndef HPP_GUARD_AIS4104_APPLICATION_CONFIGURATION_H
#define HPP_GUARD_AIS4104_APPLICATION_CONFIGURATION_H

#include "document_store.h"

#include <praxis/scene/log_buffer.h>
#include <praxis/scene/visualizer.h>

#include <praxis/config/error.h>
#include <praxis/config/store.h>
#include <praxis/config/writer.h>
#include <praxis/config/document.h>
#include <praxis/config/declaration.h>

#include <praxis/compat/expected.h>

#include <string>
#include <vector>

namespace ais4104 {

praxis::config::declaration preferences_keyspace();

praxis::scene::visualizer::geometry preferred_geometry(const praxis::config::document &values);

praxis::scene::severity preferred_level(const praxis::config::document &values);

std::vector<praxis::config::edit> preferences_edits(const praxis::scene::visualizer::geometry &left, praxis::scene::severity level);

inline constexpr const char *preset_instances = "presets/preset";
inline constexpr const char *document_leaf    = "document";

praxis::config::declaration application_keyspace();

std::vector<std::string> preset_keys(const praxis::config::document &values);

praxis::expected<praxis::config::location, praxis::config::error> preset_document(const praxis::config::document &values, const std::string &key, const document_store &mine);

std::vector<praxis::config::location> preset_locations(const praxis::config::document &values, const document_store &mine);

}

#endif

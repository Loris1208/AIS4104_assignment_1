#ifndef HPP_GUARD_AIS4104_APPLICATION_ARM_RENDER_LOOP_H
#define HPP_GUARD_AIS4104_APPLICATION_ARM_RENDER_LOOP_H

#include "write_back.h"

#include <praxis/scene/log_buffer.h>
#include <praxis/scene/visualizer.h>
#include <praxis/scene/preset_registry.h>

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <filesystem>

namespace ais4104 {

std::optional<praxis::scene::visualizer::geometry> run_and_report(const std::shared_ptr<praxis::scene::preset_registry> &registry, const std::shared_ptr<write_back> &writing,
                                                                  const std::shared_ptr<praxis::scene::log_buffer> &messages, const std::vector<std::string> &opened,
                                                                  const praxis::scene::visualizer::geometry &window, const std::filesystem::path &root);

}

#endif

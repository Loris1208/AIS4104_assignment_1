#ifndef HPP_GUARD_AIS4104_APPLICATION_OFFERED_ARMS_H
#define HPP_GUARD_AIS4104_APPLICATION_OFFERED_ARMS_H

#include "arm.h"
#include "write_back.h"
#include "document_store.h"

#include <praxis/manipulator/capabilities.h>

#include <praxis/trajectory/capabilities.h>

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/optimization/capabilities.h>

#include <praxis/scene/preset_registry.h>

#include <praxis/config/document.h>

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace ais4104 {

struct offered_capabilities
{
    praxis::manipulator::capabilities arm;
    praxis::trajectory::capabilities shapes;
    praxis::rigid_motion::capabilities motions;
    praxis::optimization::capabilities solving;
};

std::vector<std::string> register_offered_arms(const std::shared_ptr<praxis::scene::preset_registry> &registry, const document_store &mine, const praxis::config::document &values,
                                               const std::shared_ptr<write_back> &writing, const std::filesystem::path &packages, const std::filesystem::path &assets,
                                               const offered_capabilities &offered, tool_offset_source offsets);

}

#endif

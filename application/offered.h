#ifndef HPP_GUARD_AIS4104_APPLICATION_OFFERED_H
#define HPP_GUARD_AIS4104_APPLICATION_OFFERED_H

#include "write_back.h"
#include "document_store.h"

#include <praxis/rigid_motion/capabilities.h>

#include <praxis/scene/preset_registry.h>

#include <praxis/config/document.h>

#include <memory>
#include <string>
#include <vector>

namespace ais4104 {

std::vector<std::string> register_offered(const std::shared_ptr<praxis::scene::preset_registry> &registry, const document_store &mine, const praxis::config::document &values,
                                          const std::shared_ptr<write_back> &writing, const praxis::rigid_motion::capabilities &motions);

}

#endif

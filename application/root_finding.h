#ifndef HPP_GUARD_AIS4104_APPLICATION_ROOT_FINDING_H
#define HPP_GUARD_AIS4104_APPLICATION_ROOT_FINDING_H

#include <praxis/optimization/registered.h>

#include <vector>

namespace ais4104 {

praxis::optimization::registered_curve two_roots_curve();

praxis::optimization::registered_curve cubic_curve();

praxis::optimization::registered_curve cycle_curve();

praxis::optimization::registered_curve arctangent_curve();

praxis::optimization::registered_surface unit_circle_surface();

praxis::optimization::registered_surface two_link_surface();

std::vector<praxis::optimization::registered_curve> root_finding_curves();

std::vector<praxis::optimization::registered_surface> root_finding_surfaces();

}

#endif

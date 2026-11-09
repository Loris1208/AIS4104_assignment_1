#ifndef HPP_GUARD_AIS4104_APPLICATION_MINIMIZATION_H
#define HPP_GUARD_AIS4104_APPLICATION_MINIMIZATION_H

#include <praxis/optimization/registered.h>

#include <vector>

namespace ais4104 {

praxis::optimization::registered_curve parabola_curve();

praxis::optimization::registered_curve quartic_curve();

praxis::optimization::registered_surface elongated_bowl_surface();

praxis::optimization::registered_surface rosenbrock_surface();

std::vector<praxis::optimization::registered_curve> minimization_curves();

std::vector<praxis::optimization::registered_surface> minimization_surfaces();

}

#endif

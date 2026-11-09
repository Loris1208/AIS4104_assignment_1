#ifndef HPP_GUARD_AIS4104_OPTIMIZATION_CAPABILITIES_H
#define HPP_GUARD_AIS4104_OPTIMIZATION_CAPABILITIES_H

#include <praxis/optimization/solver.h>
#include <praxis/optimization/minimizer.h>

namespace ais4104::optimization {

praxis::optimization::solver_ops solvers();

praxis::optimization::minimizer_ops minimizers();

}

#endif

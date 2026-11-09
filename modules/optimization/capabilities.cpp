#include "optimization/solvers.h"
#include "optimization/capabilities.h"

#include <praxis/optimization/capabilities.h>

namespace ais4104::optimization {

namespace {

constexpr praxis::optimization::solver_ops bound_solver{
        .solve = &newton_raphson,
};

}

praxis::optimization::solver_ops solvers()
{
    return bound_solver;
}

praxis::optimization::minimizer_ops minimizers()
{
    praxis::optimization::minimizer_ops bound = praxis::optimization::baseline().minimizer;
    bound.gradient_descent                    = &gradient_descent;

    return bound;
}

}

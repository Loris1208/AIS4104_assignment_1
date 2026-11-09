#ifndef HPP_GUARD_AIS4104_APPLICATION_FUNCTIONS_H
#define HPP_GUARD_AIS4104_APPLICATION_FUNCTIONS_H

#include <praxis/optimization/registered.h>

#include <vector>

namespace ais4104 {

struct registered_functions
{
    std::vector<praxis::optimization::registered_curve> curves;
    std::vector<praxis::optimization::registered_surface> surfaces;
};

struct offered_function_forms
{
    registered_functions root;
    registered_functions minimum;
};

offered_function_forms offered_functions();

}

#endif

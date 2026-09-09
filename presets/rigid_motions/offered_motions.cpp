#include "motions.h"

#include <praxis/rigid_motion/capabilities.h>

namespace ais4104 {

praxis::rigid_motion::capabilities offered_motions()
{
    return praxis::rigid_motion::baseline();
}

}

#include "motions.h"

#include "rigid_motion/capabilities.h"

#include <praxis/rigid_motion/capabilities.h>

namespace ais4104 {

praxis::rigid_motion::capabilities offered_motions()
{
    return rigid_motion::motions();
}

}

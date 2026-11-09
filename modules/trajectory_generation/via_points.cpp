//Assignment 3
#include "trajectory_generation/via_points.h"

namespace ais4104::trajectory {

//TASK: T.5a
//REFERENCE: Equations (9.26, 9.27, 9.28 and 9.29) page 337, MR pre-print 2019
praxis::expected<cubic_segment, praxis::refusal> segment_coefficients(const praxis::trajectory::via_point &from, const praxis::trajectory::via_point &to)
{
    cubic_segment coef;
    double deltat = to.time - from.time;
    coef.a0 = from.position;
    coef.a1 = from.velocity;
    coef.a2 = (3 * to.position - 3 * from.position - 2 * from.velocity * deltat - to.velocity * deltat)/std::pow(deltat, 2);
    coef.a3 = (2 * from.position + (from.velocity + to.velocity) * deltat - 2 * to.position)/std::pow(deltat, 3);
    return coef;
}

//TASK: T.5b
//REFERENCE: Equation (9.25) page 337, MR pre-print 2019
praxis::expected<praxis::trajectory::trajectory_sample, praxis::refusal> segment_sample(const cubic_segment &segment, double dt)
{
    praxis::trajectory::trajectory_sample sample;
    sample.position = segment.a0 + segment.a1 * dt + segment.a2 * std::pow(dt, 2) + segment.a3 * std::pow(dt, 3);;
    sample.velocity = segment.a1 + 2 * segment.a2 * dt + 3 * segment.a3 * std::pow(dt, 2);
    sample.acceleration = 2 * segment.a2 + 6 * segment.a3 * dt;
    return sample;
}

}

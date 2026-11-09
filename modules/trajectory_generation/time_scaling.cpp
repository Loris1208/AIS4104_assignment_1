//Assignment 3
#include "trajectory_generation/time_scaling.h"

#include <cmath>

namespace ais4104::trajectory {

//TASK: T.4a
//REFERENCE: Equations (9.9) and (9.10)  page 331, MR pre-print 2019
praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> cubic(double t, double duration)
{
    double a0, a1, a2, a3, s, ds, dds;
    praxis::trajectory::scaling_sample sample{s,ds,dds};
    a0 = 0;
    a1 = 0;
    a2 = 3/(std::pow(duration,2));
    a3 = -2/(std::pow(duration,3));
    sample.s = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3);
    sample.ds = a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2);
    sample.dds = 2 * a2 + 6 * a3 * t;
    return sample;
}

//TASK: T.4b
//REFERENCE: Section (9.2.2.1) and exercise 9.5 page 331, MR pre-print 2019
praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> quintic(double t, double duration)
{
    double a0, a1, a2, a3, a4, a5, a6, s, ds, dds;
    praxis::trajectory::scaling_sample sample{s,ds,dds};
    a0 = 0;
    a1 = 0;
    a2 = 0;
    a3 = 10/(std::pow(duration,3));
    a4 = -15/(std::pow(duration,4));
    a5 = 6/(std::pow(duration,5));
    sample.s = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
    sample.ds = a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) + 5 * a5 * std::pow(t, 4);
    sample.dds = 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) + 20 * a5 * std::pow(t, 3);;
    return sample;
}

}

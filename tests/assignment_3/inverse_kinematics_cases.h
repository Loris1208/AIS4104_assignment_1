#ifndef HPP_GUARD_AIS4104_ASSIGNMENT_3_INVERSE_KINEMATICS_CASES_H
#define HPP_GUARD_AIS4104_ASSIGNMENT_3_INVERSE_KINEMATICS_CASES_H

#include "suite_rows.h"

#include <praxis/manipulator/types.h>
#include <praxis/manipulator/kinematics.h>
#include <praxis/manipulator/screw_chain.h>

#include <praxis/rigid_motion/types.h>

#include <Eigen/Core>

#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

namespace ais4104 {

constexpr std::size_t most_joints = 7;

// The home pose row by row and each screw angular part first, in meters; the limits in radians and their first and second time derivatives; entries past joints are zero.
struct frozen_chain
{
    std::size_t joints;
    std::array<double, 16> home;
    std::array<std::array<double, 6>, most_joints> space_screws;
    std::array<double, most_joints> velocity;
    std::array<double, most_joints> acceleration;
    std::array<double, most_joints> lower_position;
    std::array<double, most_joints> upper_position;
};

constexpr std::array<frozen_chain, 3> frozen_chains{
        frozen_chain{.joints         = 6,
                     .home           = {2.220446049250313e-16, 0.0, 1.0, 0.98, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 2.220446049250313e-16, 0.43500000000000005, 0.0, 0.0, 0.0, 1.0},
                     .space_screws   = {{{0.0, 0.0, -1.0, 0.0, 0.0, 0.0},
                                         {0.0, 1.0, 0.0, -0.4, 0.0, 0.025},
                                         {0.0, 1.0, 0.0, -0.4, 0.0, 0.48000000000000004},
                                         {-1.0, 0.0, 0.0, 0.0, -0.43500000000000005, 0.0},
                                         {0.0, 1.0, 0.0, -0.43500000000000005, 0.0, 0.9},
                                         {-1.0, 0.0, 0.0, 0.0, -0.43500000000000005, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
                     .velocity       = {6.283185307179586, 5.235987755982989, 6.283185307179586, 6.649704450098396, 6.771877497737998, 10.733774899765127, 0.0},
                     .acceleration   = {3.141592653589793, 2.6179938779914944, 3.141592653589793, 3.324852225049198, 3.385938748868999, 5.366887449882563, 0.0},
                     .lower_position = {-2.9670597283903604, -3.3161255787892263, -2.0943951023931953, -3.2288591161895095, -2.0943951023931953, -6.1086523819801535, 0.0},
                     .upper_position = {2.9670597283903604, 0.7853981633974483, 2.722713633111154, 3.2288591161895095, 2.0943951023931953, 6.1086523819801535, 0.0}},
        frozen_chain{
                .joints         = 7,
                .home           = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.3054999999999999, 0.0, 0.0, 0.0, 1.0},
                .space_screws   = {{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0, -0.3595, 0.0, 0.0},
                                    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                    {0.0, -1.0, 0.0, 0.7795000000000001, 0.0, 0.0},
                                    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0, -1.1795, 0.0, 0.0},
                                    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}}},
                .velocity       = {1.4835298641951802, 1.4835298641951802, 1.7453292519943295, 1.3089969389957472, 2.2689280275926285, 2.356194490192345, 2.356194490192345},
                .acceleration   = {0.7417649320975901, 0.7417649320975901, 0.8726646259971648, 0.6544984694978736, 1.1344640137963142, 1.1780972450961724, 1.1780972450961724},
                .lower_position = {-2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.0943951023931953, -3.0543261909900763},
                .upper_position = {2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.0943951023931953, 3.0543261909900763}},
        frozen_chain{.joints         = 2,
                     .home           = {1.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                     .space_screws   = {{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 1.0, 0.0, -1.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
                     .velocity       = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     .acceleration   = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     .lower_position = {-3.141592653589793, -3.141592653589793, 0.0, 0.0, 0.0, 0.0, 0.0},
                     .upper_position = {3.141592653589793, 3.141592653589793, 0.0, 0.0, 0.0, 0.0, 0.0}},
};

constexpr std::array<std::string_view, 3> subject_names{"kr6", "lbr_med", "example_6_1"};

// The target pose row by row, its position in meters; the seed in radians, zero past the joints.
struct ik_case
{
    std::size_t subject;
    std::array<double, 16> target;
    std::array<double, most_joints> seed;
};

constexpr std::array<ik_case, 17> ik_cases{
        ik_case{.subject = 0,
                .target  = {0.9586135941640198, -0.28248123593898355, 0.035557396225325756, 0.5049071997967832, 0.10014983928226355, 0.4514703044547102, 0.8866479424705774,
                            0.20397840128282096, -0.26651451513130997, -0.846391703372591, 0.46107602159119826, 1.1319551726296042, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-0.229775372655031, -0.8455823675905029, -0.6410353520554966, -1.9084159227397697, 1.2430210282475151, 1.9395676527113932, 0.0}},
        ik_case{.subject = 0,
                .target  = {0.8978115732318364, 0.4365569949291554, -0.057899647230362734, -0.15738632298746957, 0.3240843669674988, -0.5659634398858371, 0.7580598312797389,
                            0.6168002062618336, 0.29816723840531273, -0.6993592602454163, -0.6496098237024541, 0.649489083201216, 0.0, 0.0, 0.0, 1.0},
                .seed    = {1.6988071308797894, -2.1064042511804253, -1.5289415511504143, 0.17995134235844568, -0.6292876856381348, -1.9080967747254318, 0.0}},
        ik_case{.subject = 0,
                .target  = {0.9122849020751485, 0.2374893483668324, -0.3336690978470045, 0.4075968267354597, 0.333585428409386, -0.9035375094336768, 0.26896232263890546,
                            -0.21729384681248975, -0.23760685890492106, -0.3566774151228049, -0.9035066143325412, 0.2732475168362155, 0.0, 0.0, 0.0, 1.0},
                .seed    = {0.5983239139542217, -1.0518835210236717, 2.0772887083813294, 2.8169184668977856, -0.884514837836649, 0.7388461392242273, 0.0}},
        ik_case{.subject = 0,
                .target  = {-0.667162375344757, -0.6912682442183182, -0.2775654507673189, 0.05504885878759502, 0.7434114777017747, -0.5942286862191326, -0.3069717304501293,
                            0.04464040916949852, 0.04726245598363447, -0.4111453307646907, 0.9103437687186023, 1.271672946221511, 0.0, 0.0, 0.0, 1.0},
                .seed    = {2.5314814525859743, -2.2200396628249175, 0.565931612150504, -0.8031459788251332, 0.4114867687279393, 2.4851142694727244, 0.0}},
        ik_case{.subject = 0,
                .target  = {-0.24075766920213454, -0.22481726491147688, 0.9441890393973434, 0.26413878500474286, 0.7347654897399204, -0.6778241695480904, 0.025962863164456937,
                            0.2511561867286853, 0.6341572516400097, 0.7000082803611428, 0.3283793349439415, 0.5010813193380919, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-1.3537342512772306, -1.3407989822723896, 2.0808290772620825, 1.9576195300313475, 1.5063922140189105, 1.3078025858106765, 0.0}},
        ik_case{.subject = 0,
                .target  = {-0.5479994398057898, -0.46729578744148426, 0.6937804126753535, 0.18027730506487094, -0.4063253818731923, 0.8736873066432537, 0.2675260253808248,
                            -0.08030175987595811, -0.7311609248435976, -0.1352964790742927, -0.6686542938858273, 1.0233370867876068, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-2.108255424597146, -0.9844331144462126, -1.6762771059923551, 2.4094096406924495, 1.9415973763735785, 0.9037279487367139, 0.0}},
        ik_case{.subject = 0,
                .target  = {0.4200142578292579, -0.8862060660934716, 0.19551683211240659, -0.26122065067052863, -0.8448908769582439, -0.4604920055918895, -0.2722251252524069,
                            -0.11981521700434575, 0.3312814954881529, -0.05085195379818257, -0.9421606283124127, 0.061619355072421556, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-0.4340485591438835, -3.240612363395962, -1.6258263492776877, 1.0739900650077612, 0.34146673293272845, 0.22273639464825956, 0.0}},
        ik_case{.subject = 0,
                .target  = {0.5861458452279933, 0.15528771279488596, 0.7951847422938234, 0.04910409457602671, -0.04831352326639778, 0.986413126019997, -0.1570189424402186,
                            0.40870842715782457, -0.8087637798464696, 0.053617824185530356, 0.5856844520200797, 0.7952137364098929, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-1.844211492956925, -0.02882184447668834, -1.920880529042333, 1.3798435388279264, 1.3861260632592827, -0.011522619007407031, 0.0}},
        ik_case{.subject = 1,
                .target  = {-0.1095067050197037, -0.5387620240269789, -0.8353105787801809, -0.46144976678237104, 0.6059184746354809, -0.7023565609012394, 0.3735747093345531,
                            0.12830711819618928, -0.7879537319228457, -0.4652211762434475, 0.403358616523217, 1.0539652216464312, 0.0, 0.0, 0.0, 1.0},
                .seed    = {2.139230934208796, 0.12560215263663177, 1.5576065400442904, -1.3140199563481156, -1.3199236552985854, 1.0635673113878583, 0.17458337930446177}},
        ik_case{.subject = 1,
                .target  = {0.007457830430644491, 0.7523990751222254, 0.6586653266420568, -0.7187985716796903, 0.4885600081228807, -0.577458016501075, 0.65410347624945,
                            0.13493795177838475, 0.8724984236250737, 0.31691934452453974, -0.3718986822212437, 0.22679334722722533, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-0.011281601736420566, -1.6661222934866844, -0.870824899138155, 0.061861517446256564, -0.5676878505755234, -1.5937201337279805, 1.6361470268899991}},
        ik_case{.subject = 1,
                .target  = {0.4665314321364241, -0.4504165268458395, 0.7612314859310984, -0.18300595082790494, -0.7495570285184332, 0.25558687269846136, 0.6106059379852419,
                            0.19833090691785304, -0.4695877807474911, -0.855453273328445, -0.21837356370287758, 1.062810174537431, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-2.5420876311844247, -0.2006193986479008, 2.1888219022764304, 0.7509838837551883, 1.9752562632992314, 1.8660731724807547, -0.41924707427160524}},
        ik_case{.subject = 1,
                .target  = {-0.8558521413748614, 0.198343234792851, -0.4776788390916773, -0.13888487852671666, -0.5167929972712342, -0.3654748147803494, 0.7741790217596209,
                            -0.01928191612186203, -0.02102641375695491, 0.9094438525676984, 0.41529479764541233, 1.0916697190432896, 0.0, 0.0, 0.0, 1.0},
                .seed    = {1.7037000000582698, 0.4256553974329088, 2.4683504123848263, -1.4233223128864352, 0.2074746272667204, -1.3158649217546783, -0.6947618867612795}},
        ik_case{.subject = 1,
                .target  = {0.6601658899832881, -0.36051721548319426, -0.6589448649491223, 0.41333088624236425, 0.4235968910914848, 0.9031614953148625, -0.06974946048716364,
                            0.04546737979126929, 0.6202795108337869, -0.23308078153478395, 0.7489503840168134, 0.7785633724114328, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-2.257538883762794, -1.0106643253390604, -1.9415961573210425, 1.7896580772842747, -1.2197617462496901, -1.1887825782455348, 0.7946870420578154}},
        ik_case{.subject = 1,
                .target  = {-0.6141439945603652, -0.0928570135212552, -0.7837121467639453, -0.7955752466709773, 0.7883415095449735, -0.02603788116772433, -0.6146866625140387,
                            -0.17320227966907287, 0.0366717639852596, -0.9953389391479714, 0.08919404656141863, 0.37330053121019335, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-0.782754406010833, -1.4511600732022796, 2.2912307873119584, 0.7819596738495302, -1.8448564374600165, -0.42905135133449246, 1.4776007348567677}},
        ik_case{.subject = 1,
                .target  = {-0.18895495131798296, -0.9761157769753663, 0.10721015022933089, -0.1530972337637215, 0.7524429641119093, -0.073769274601648, 0.654513315283372,
                            0.5738999730630046, -0.6309719582762, 0.20434305484778217, 0.7484105182348613, 0.9200268472678096, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-0.42369476240383086, -1.0287535328886923, 1.4004028807602498, -0.6352037609652664, -2.1729538677095364, 0.46985826516796414, -2.8517045790311135}},
        ik_case{.subject = 1,
                .target  = {0.7749316083858044, 0.6281333144022418, -0.07021069478821806, -0.018159068140058326, -0.5356515172751493, 0.5937164059911189, -0.6004817077129156,
                            -0.5704406459233264, -0.33549732393183174, 0.5029407207564757, 0.7965501723303663, 1.1030069114434529, 0.0, 0.0, 0.0, 1.0},
                .seed    = {-1.7937112215867914, 0.6572844681356425, 0.3797158283164652, 0.6503701269695156, 2.9593255968625933, 0.17188304953137468, -1.6857998285553681}},
        ik_case{.subject = 2,
                .target  = {-0.49999999999999983, -0.8660254037844387, 0.0, 0.36602540378443893, 0.8660254037844387, -0.49999999999999983, 0.0, 1.3660254037844386, 0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 1.0},
                .seed    = {0.0, 0.5235987755982988, 0.0, 0.0, 0.0, 0.0, 0.0}},
};

// Modern Robotics, Example 6.1: meters, then radians.
constexpr double book_position_tolerance    = 1e-04;
constexpr double book_orientation_tolerance = 0.001;

constexpr std::uint32_t solver_iteration_cap = 100;
constexpr std::uint32_t solver_limit_cap     = 1;

// Radians, then meters.
constexpr double reach_radians = 0.0012;
constexpr double reach_metres  = 0.0017;

constexpr pinned_bound reach_bound{reach_radians, reach_metres, bound_standing::above_the_platform, bound_standing::above_the_platform};

// The step in radians, the errors in radians or meters, the end pose in radians and meters; a relative tolerance is dimensionless.
constexpr double record_step_absolute  = 1e-12;
constexpr double record_step_relative  = 0.0;
constexpr double record_error_absolute = 1e-12;
constexpr double record_error_relative = 5.203418353640438e-08;
constexpr double record_end_radians    = 1e-12;
constexpr double record_end_metres     = 1e-12;

constexpr bool last_row_affine(const std::array<double, 16> &pose)
{
    return pose[12] == 0.0 && pose[13] == 0.0 && pose[14] == 0.0 && pose[15] == 1.0;
}

constexpr bool limits_ordered(const frozen_chain &chain)
{
    for(std::size_t joint = 0; joint < chain.joints; ++joint)
        if(!(chain.lower_position[joint] < chain.upper_position[joint]))
            return false;
    return true;
}

constexpr bool seed_inside_limits(const ik_case &row)
{
    const frozen_chain &chain = frozen_chains[row.subject];
    for(std::size_t joint = 0; joint < most_joints; ++joint)
    {
        const double theta = row.seed[joint];
        const bool held    = joint < chain.joints ? chain.lower_position[joint] <= theta && theta <= chain.upper_position[joint] : theta == 0.0;
        if(!held)
            return false;
    }
    return true;
}

constexpr std::size_t cases_of(std::size_t subject)
{
    return static_cast<std::size_t>(std::ranges::count_if(ik_cases, [subject](const ik_case &row) { return row.subject == subject; }));
}

static_assert(frozen_chains.size() == subject_names.size());
static_assert(frozen_chains[0].joints == 6 && frozen_chains[1].joints == 7 && frozen_chains[2].joints == 2);
static_assert(std::ranges::all_of(frozen_chains, limits_ordered));
static_assert(std::ranges::all_of(frozen_chains, [](const frozen_chain &chain) { return last_row_affine(chain.home); }));
static_assert(ik_cases.size() == 17);
static_assert(std::ranges::all_of(ik_cases, [](const ik_case &row) { return row.subject < frozen_chains.size(); }));
static_assert(cases_of(0) == 8 && cases_of(1) == 8 && cases_of(2) == 1);
static_assert(std::ranges::all_of(ik_cases, seed_inside_limits));
static_assert(std::ranges::all_of(ik_cases, [](const ik_case &row) { return last_row_affine(row.target); }));

constexpr std::array<std::size_t, 17> solver_limit_cases{
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
};

static_assert(std::ranges::all_of(solver_limit_cases, [](std::size_t index) { return index < ik_cases.size(); }));

inline Eigen::Matrix4d home_of(const frozen_chain &chain)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(chain.home.data());
}

inline std::vector<praxis::screw_axis> space_screws_of(const frozen_chain &chain)
{
    std::vector<praxis::screw_axis> screws(chain.joints);
    for(std::size_t joint = 0; joint < chain.joints; ++joint)
        screws[joint] = Eigen::Map<const praxis::screw_axis>(chain.space_screws[joint].data());

    return screws;
}

inline Eigen::VectorXd leading_entries(const std::array<double, most_joints> &values, std::size_t count)
{
    return Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(count));
}

inline praxis::manipulator::joint_limits limits_of(const frozen_chain &chain)
{
    return praxis::manipulator::joint_limits{.velocity       = leading_entries(chain.velocity, chain.joints),
                                             .acceleration   = leading_entries(chain.acceleration, chain.joints),
                                             .lower_position = leading_entries(chain.lower_position, chain.joints),
                                             .upper_position = leading_entries(chain.upper_position, chain.joints)};
}

inline praxis::manipulator::screw_chain chain_of(const frozen_chain &chain)
{
    return praxis::manipulator::screw_chain(home_of(chain), space_screws_of(chain), limits_of(chain));
}

inline Eigen::Matrix4d target_of(const ik_case &row)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(row.target.data());
}

inline Eigen::VectorXd seed_of(const ik_case &row)
{
    return leading_entries(row.seed, frozen_chains[row.subject].joints);
}

inline praxis::manipulator::solver_parameters solver_route_parameters()
{
    return praxis::manipulator::solver_parameters(book_position_tolerance, book_orientation_tolerance, solver_iteration_cap);
}

}

#endif

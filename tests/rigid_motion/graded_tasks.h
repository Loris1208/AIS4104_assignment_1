#ifndef HPP_GUARD_AIS4104_RIGID_MOTION_GRADED_TASKS_H
#define HPP_GUARD_AIS4104_RIGID_MOTION_GRADED_TASKS_H

#include <array>
#include <cstddef>
#include <string_view>

namespace ais4104 {

// The slot is praxis's own, in praxis's own order; the task is the identifier the assignment prints
// for it, which indexes nothing.
struct graded_task
{
    std::string_view task;
    std::string_view slot;
};

inline constexpr std::array graded_tasks{
        graded_task{"1j", "frame.euler_from_rotation_matrix"},
        graded_task{"1d", "frame.rotate_x"},
        graded_task{"1e", "frame.rotate_y"},
        graded_task{"1f", "frame.rotate_z"},
        graded_task{"1g", "frame.rotation_matrix_from_frame_axes"},
        graded_task{"1i", "frame.rotation_matrix_from_euler"},
        graded_task{"1h", "frame.rotation_matrix_from_axis_angle"},
        graded_task{"2b", "frame.rotation_matrix_from_transform"},
        graded_task{"2d", "frame.transformation_matrix_from_position"},
        graded_task{"2e", "frame.transformation_matrix_from_rotation"},
        graded_task{"2c", "frame.transformation_matrix_from_rotation_position"},
        graded_task{"2f", "frame.inverse"},
        graded_task{"1b", "screw.skew_symmetric"},
        graded_task{"1c", "screw.from_skew_symmetric"},
        graded_task{"3f", "screw.adjoint_matrix_from_rotation_position"},
        graded_task{"3g", "screw.adjoint_matrix_from_transform"},
        graded_task{"3h", "screw.adjoint_map"},
        graded_task{"3d", "screw.twist_from_angular_linear"},
        graded_task{"3e", "screw.twist_from_screw"},
        graded_task{"3i", "screw.twist_matrix_from_angular_linear"},
        graded_task{"3j", "screw.twist_matrix_from_twist"},
        graded_task{"3c", "screw.screw_axis_from_angular_linear"},
        graded_task{"3b", "screw.screw_axis_from_point_direction_pitch"},
        graded_task{"3k", "screw.matrix_exponential_so3"},
        graded_task{"3l", "screw.matrix_exponential_se3"},
        graded_task{"3m", "screw.matrix_exponential_screw"},
        graded_task{"3n", "screw.matrix_logarithm_so3"},
        graded_task{"3o", "screw.matrix_logarithm_se3_rp"},
        graded_task{"3p", "screw.matrix_logarithm_se3"},
};

constexpr const graded_task *graded_task_of(std::string_view task)
{
    for(const graded_task &entry : graded_tasks)
        if(entry.task == task)
            return &entry;

    return nullptr;
}

constexpr std::size_t tasks_named(std::string_view prefix)
{
    std::size_t named = 0;
    for(const graded_task &entry : graded_tasks)
        named += static_cast<std::size_t>(entry.slot.starts_with(prefix));

    return named;
}

static_assert(tasks_named("frame.") + tasks_named("screw.") == graded_tasks.size());

template<typename Rows>
constexpr std::size_t rows_naming(const Rows &rows, std::string_view task)
{
    std::size_t named = 0;
    for(const auto &row : rows)
        named += static_cast<std::size_t>(row.task == task);

    return named;
}

// A runner table covers a slot prefix when it names every task of that prefix exactly once and
// nothing else, which is what lets two units partition the table between them.
template<typename Rows>
constexpr bool tasks_cover(const Rows &rows, std::string_view prefix)
{
    if(rows.size() != tasks_named(prefix))
        return false;

    for(const graded_task &entry : graded_tasks)
        if(entry.slot.starts_with(prefix) && rows_naming(rows, entry.task) != 1)
            return false;

    return true;
}

template<typename Rows>
constexpr bool tasks_are_distinct(const Rows &rows)
{
    for(const auto &row : rows)
        if(graded_task_of(row.task) == nullptr || rows_naming(rows, row.task) != 1)
            return false;

    return true;
}

}

#endif

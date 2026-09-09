#include "graded_tasks.h"

#include <catch2/catch_test_macros.hpp>

#include <cctype>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <string_view>

#ifndef AIS4104_MODULE_SOURCE_DIRECTORY
    #define AIS4104_MODULE_SOURCE_DIRECTORY "modules/rigid_motion"
#endif

namespace {

constexpr std::string_view task_marker = "//TASK: ";

struct marked_task
{
    std::string name;
    std::string task;
};

std::string identifier_before_parenthesis(const std::string &line)
{
    const std::size_t opened = line.find('(');
    if(opened == std::string::npos)
        return {};

    std::size_t first = opened;
    while(first > 0 && (std::isalnum(static_cast<unsigned char>(line[first - 1])) != 0 || line[first - 1] == '_'))
        --first;

    return line.substr(first, opened - first);
}

std::string task_identifier(const std::string &line)
{
    const std::string carried = line.substr(task_marker.size());

    return carried.substr(0, carried.find(' '));
}

std::vector<marked_task> markers_in(const std::string &source)
{
    std::ifstream opened(source);
    INFO(source);
    REQUIRE(opened.is_open());

    std::vector<marked_task> marked;
    std::string line;
    while(std::getline(opened, line))
    {
        if(!line.starts_with(task_marker))
            continue;

        const std::string task = task_identifier(line);
        while(std::getline(opened, line) && line.starts_with("//"))
            continue;

        marked.push_back(marked_task{identifier_before_parenthesis(line), task});
    }

    return marked;
}

// A marker is held to the table through the function it stands above, never through its position in
// the file, so the seam is free to be ordered however the assignment asks for it.
void require_markers_match(std::string_view file, std::string_view extension)
{
    const std::string prefix              = std::string{extension} + ".";
    const std::vector<marked_task> marked = markers_in(std::string{AIS4104_MODULE_SOURCE_DIRECTORY} + "/" + std::string{file});

    for(const marked_task &carried : marked)
    {
        INFO(std::format("{} marks task {} above {}", file, carried.task, carried.name));

        const ais4104::graded_task *const entry = ais4104::graded_task_of(carried.task);
        REQUIRE(entry != nullptr);
        REQUIRE(std::string{entry->slot} == prefix + carried.name);
    }

    for(const ais4104::graded_task &entry : ais4104::graded_tasks)
    {
        if(!entry.slot.starts_with(prefix))
            continue;

        INFO(std::format("{} must mark {} once", file, entry.slot));
        REQUIRE(std::count_if(marked.begin(), marked.end(), [&](const marked_task &carried) { return std::string_view{carried.task} == entry.task; }) == 1);
    }
}

}

TEST_CASE("every graded task is the marker the seam file it belongs to carries", "[rigid_motion][grading]")
{
    require_markers_match("frame.cpp", "frame");
    require_markers_match("screw.cpp", "screw");
}

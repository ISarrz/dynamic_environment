#include "StaticTests.h"

#include "DataPaths.h"
#include "Astar.h"
#include "DstarLite.h"
#include "Field.h"
#include "heuristic_functions.h"

#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

struct RunStats {
    int success = 0;
    int no_path = 0;
    int exceptions = 0;
    double total_ms = 0.0;
    double min_ms = std::numeric_limits<double>::infinity();
    double max_ms = 0.0;

    void Add(const std::string &status, const double ms) {
        if (status == "Path found") {
            ++success;
        } else if (status == "No path") {
            ++no_path;
        } else {
            ++exceptions;
        }

        total_ms += ms;
        if (ms < min_ms) {
            min_ms = ms;
        }
        if (ms > max_ms) {
            max_ms = ms;
        }
    }

    int TotalRuns() const { return success + no_path + exceptions; }
};

void PrintStats(const std::string &label, const RunStats &stats) {
    const int total_runs = stats.TotalRuns();
    const double avg_ms = total_runs > 0 ? stats.total_ms / total_runs : 0.0;
    const double min_ms = total_runs > 0 ? stats.min_ms : 0.0;
    const double max_ms = total_runs > 0 ? stats.max_ms : 0.0;

    std::cout << std::left << std::setw(14) << label << " | "
              << "ok=" << std::setw(4) << stats.success << " "
              << "no_path=" << std::setw(4) << stats.no_path << " "
              << "exceptions=" << std::setw(4) << stats.exceptions << " "
              << std::fixed << std::setprecision(3)
              << "avg=" << std::setw(9) << avg_ms << "ms "
              << "min=" << std::setw(9) << min_ms << "ms "
              << "max=" << std::setw(9) << max_ms << "ms\n";
}

int TestAstar(Field field, int x1, int y1, int x2, int y2) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};

    auto result = Astar(start_p, end_p, field, ManhattanDistance);
    if (!result.has_value()) {
        return -1;
    }

    return 0;
}

int TestDstarLite(Field field, int x1, int y1, int x2, int y2) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    DstarLite dstar_lite = DstarLite(&field, start_p, end_p, ManhattanDistance);
    dstar_lite.ComputeShortestPath();

    Coordinates current_pos = start_p;

    while (current_pos != end_p) {
        auto next_node = dstar_lite.GetNextNode();
        if (!next_node.has_value()) {
            return -1;
        }
        dstar_lite.MoveStart({next_node->first, next_node->second});
        current_pos = {next_node->first, next_node->second};
    }

    return 0;
}

} // namespace

void GetStaticTest() {
    Field field(GetMapPath());

    std::ifstream in(GetPointsPath());
    if (!in.is_open()) {
        std::cout << "Cannot open data/points.txt\n";
        return;
    }

    int test_id = 1;
    RunStats a_star_stats;
    RunStats dstar_stats;

    std::cout << std::left << std::setw(6) << "Test" << "| "
              << std::setw(10) << "A*" << "| "
              << std::setw(10) << "A* time" << "| "
              << std::setw(10) << "D* Lite" << "| "
              << std::setw(10) << "D* time" << "\n";
    std::cout << "-------------------------------------------------------------\n";

    int x1, y1, x2, y2;
    while (in >> x1 >> y1 >> x2 >> y2) {
        std::string a_status = "Path found";
        double a_ms = 0.0;

        auto a_start = std::chrono::high_resolution_clock::now();
        try {
            int res = TestAstar(field, x1, y1, x2, y2);
            if (res == -1) {
                a_status = "No path";
            }
        } catch (const std::exception &e) {
            a_status = "Exception";
        }
        auto a_end = std::chrono::high_resolution_clock::now();
        a_ms = std::chrono::duration<double, std::milli>(a_end - a_start).count();
        a_star_stats.Add(a_status, a_ms);

        std::string d_status = "Path found";
        double d_ms = 0.0;

        auto d_start = std::chrono::high_resolution_clock::now();
        try {
            int res = TestDstarLite(field, x1, y1, x2, y2);
            if (res == -1) {
                d_status = "No path";
            }
        } catch (const std::exception &e) {
            d_status = "Exception";
        }
        auto d_end = std::chrono::high_resolution_clock::now();
        d_ms = std::chrono::duration<double, std::milli>(d_end - d_start).count();
        dstar_stats.Add(d_status, d_ms);

        std::cout << std::left << std::setw(6) << test_id << "| "
                  << std::setw(10) << a_status << "| "
                  << std::setw(10) << std::fixed << std::setprecision(3) << a_ms
                  << "| " << std::setw(10) << d_status << "| "
                  << std::setw(10) << std::fixed << std::setprecision(3) << d_ms
                  << "\n";

        ++test_id;
    }

    if (test_id == 1) {
        std::cout << "data/points.txt is empty or has invalid format.\n";
        return;
    }

    std::cout << "-------------------------------------------------------------\n";
    PrintStats("A* Static", a_star_stats);
    PrintStats("D* Static", dstar_stats);
}

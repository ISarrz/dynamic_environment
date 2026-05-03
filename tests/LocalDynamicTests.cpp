#include "LocalDynamicTests.h"

#include "DataPaths.h"
#include "Field.h"
#include "MetricsUtils.h"
#include "MPPI.h"
#include "RMPPI.h"
#include "TestGeneration.h"
#include "TestUtils.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kMaxSteps     = 50'000;
constexpr int kMaxTestCases = 100;

struct NavResult {
    int    status; // 0=found, -1=no path, -2=timeout
    std::vector<std::pair<int, int>> path;
};

struct RunStats : BaseRunStats {
    int    timeout      = 0;
    long long total_steps = 0;
    int    min_steps    = std::numeric_limits<int>::max();
    int    max_steps    = 0;
    int    step_samples = 0;

    double total_tort = 0.0, total_turn = 0.0, total_nm = 0.0;
    double total_steps_sq = 0.0;
    int    metric_samples = 0;

    void Add(const std::string& status, double ms,
             const NavResult& res, const PathMetrics& pm) {
        if (status == "Timeout") {
            ++timeout;
        } else {
            RecordStatus(status);
        }
        RecordTiming(ms);
        if (status == "Path found" && !res.path.empty()) {
            int steps = static_cast<int>(res.path.size()) - 1;
            ++step_samples;
            total_steps    += steps;
            total_steps_sq += static_cast<double>(steps) * steps;
            if (steps < min_steps) min_steps = steps;
            if (steps > max_steps) max_steps = steps;

            ++metric_samples;
            total_tort += pm.tortuosity;
            total_turn += pm.avg_turn_deg;
            total_nm   += pm.near_miss_rate;
        }
    }

    int TotalRuns() const { return BaseRunStats::TotalRuns() + timeout; }
};

void PrintStats(const std::string& label, const RunStats& s) {
    const int total = s.TotalRuns();
    const double avg_ms    = total > 0 ? s.total_ms / total : 0.0;
    const double avg_steps = s.step_samples > 0
        ? static_cast<double>(s.total_steps) / s.step_samples : 0.0;
    const double avg_steps_sq = s.step_samples > 0
        ? s.total_steps_sq / s.step_samples : 0.0;
    const double steps_std = s.step_samples > 1
        ? std::sqrt(std::max(0.0, avg_steps_sq - avg_steps * avg_steps)) : 0.0;
    const double avg_tort = s.metric_samples > 0 ? s.total_tort / s.metric_samples : 0.0;
    const double avg_turn = s.metric_samples > 0 ? s.total_turn / s.metric_samples : 0.0;
    const double avg_nm   = s.metric_samples > 0 ? s.total_nm   / s.metric_samples : 0.0;

    std::cout << std::left << std::setw(14) << label << " | "
              << "ok=" << std::setw(4) << s.success << " "
              << "no_path=" << std::setw(4) << s.no_path << " "
              << "timeout=" << std::setw(4) << s.timeout << " "
              << std::fixed << std::setprecision(3)
              << "avg_ms=" << std::setw(8) << avg_ms << " "
              << "avg_steps=" << std::setw(7) << avg_steps << " "
              << "steps_std=" << std::setw(7) << steps_std << " "
              << "tortuosity=" << std::setw(6) << avg_tort << " "
              << "turn_deg=" << std::setw(6) << avg_turn << " "
              << "near_miss=" << std::setw(6) << avg_nm << "\n";
}

NavResult TestDynamicPlanner(
    MPPI& planner, Field field,
    int x1, int y1, int x2, int y2,
    const Changes& changes)
{
    std::pair<int, int> current = {x1, y1};
    std::pair<int, int> goal    = {x2, y2};
    int change_step    = 0;

    NavResult result;
    result.path.push_back(current);

    while (current != goal) {
        if (static_cast<int>(result.path.size()) - 1 >= kMaxSteps) {
            result.status = -2;
            return result;
        }

        if (change_step < static_cast<int>(changes.size())) {
            for (const auto& change : changes[change_step]) {
                field.Set(change.first.first,  change.first.second,  '.');
                field.Set(change.second.first, change.second.second, 'D');
            }
            ++change_step;
        }

        auto next = planner.GetNextNode(current, goal, field);
        if (!next.has_value()) {
            result.status = -1;
            return result;
        }
        current = *next;
        result.path.push_back(current);
    }
    result.status = 0;
    return result;
}

} // namespace

void LocalDynamicTests() {
    const Field   field_orig(GetMapPath());
    ChangesHeader header  = LoadChangesHeaderFromFile(GetChangesPath());
    Changes       changes = LoadChangesFromFile(GetChangesPath());
    std::ifstream in(GetPointsPath());

    if (changes.empty())
        throw std::runtime_error("data/changes.txt does not contain any steps");
    if (header.steps != static_cast<int>(changes.size()))
        throw std::runtime_error("data/changes.txt header steps does not match data");
    if (!in.is_open())
        throw std::runtime_error("Cannot open data/points.txt");

    std::cout << "Detected from header: steps=" << header.steps
              << ", objects_per_step=" << header.objects_per_step << "\n";

    MPPIConfig  mppi_cfg;
    RMPPIConfig rmppi_cfg;
    MPPI  mppi(mppi_cfg,   42);
    RMPPI rmppi(rmppi_cfg, 137);

    RunStats mppi_stats, rmppi_stats;
    int test_id = 1;

    // Column layout (pipe-separated):
    // test_id | mppi_status | mppi_ms | mppi_steps | mppi_tort | mppi_turn | mppi_nm
    //         | rmppi_status | rmppi_ms | rmppi_steps | rmppi_tort | rmppi_turn | rmppi_nm
    std::cout << std::left
              << std::setw(6)  << "Test"        << "| "
              << std::setw(10) << "MPPI"         << "| "
              << std::setw(10) << "MPPI ms"      << "| "
              << std::setw(10) << "MPPI steps"   << "| "
              << std::setw(10) << "MPPI tort"    << "| "
              << std::setw(10) << "MPPI turn"    << "| "
              << std::setw(10) << "MPPI nm"      << "| "
              << std::setw(10) << "RMPPI"        << "| "
              << std::setw(10) << "RMPPI ms"     << "| "
              << std::setw(10) << "RMPPI steps"  << "| "
              << std::setw(10) << "RMPPI tort"   << "| "
              << std::setw(10) << "RMPPI turn"   << "| "
              << std::setw(10) << "RMPPI nm"     << "\n";
    std::cout << std::string(150, '-') << "\n";
    std::cout << "(Using first " << kMaxTestCases
              << " test cases — sampling-based planners are slower per path)\n";

    int x1, y1, x2, y2;
    while (test_id <= kMaxTestCases && (in >> x1 >> y1 >> x2 >> y2)) {
        // --- MPPI ---
        NavResult   m_res;
        PathMetrics m_pm;
        double m_ms = 0.0;
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            try {
                m_res = TestDynamicPlanner(mppi, field_orig, x1, y1, x2, y2, changes);
                if (m_res.status == 0)
                    m_pm = ComputePathMetrics(m_res.path, x2, y2, field_orig);
            } catch (...) { m_res.status = -3; }
            m_ms = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - t0).count();
        }
        const std::string m_status =
            m_res.status ==  0 ? "Path found" :
            m_res.status == -1 ? "No path"    :
            m_res.status == -2 ? "Timeout"    : "Exception";
        mppi_stats.Add(m_status, m_ms, m_res, m_pm);

        // --- RMPPI ---
        NavResult   r_res;
        PathMetrics r_pm;
        double r_ms = 0.0;
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            try {
                r_res = TestDynamicPlanner(rmppi, field_orig, x1, y1, x2, y2, changes);
                if (r_res.status == 0)
                    r_pm = ComputePathMetrics(r_res.path, x2, y2, field_orig);
            } catch (...) { r_res.status = -3; }
            r_ms = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - t0).count();
        }
        const std::string r_status =
            r_res.status ==  0 ? "Path found" :
            r_res.status == -1 ? "No path"    :
            r_res.status == -2 ? "Timeout"    : "Exception";
        rmppi_stats.Add(r_status, r_ms, r_res, r_pm);

        const int m_steps = m_res.status == 0
            ? static_cast<int>(m_res.path.size()) - 1 : 0;
        const int r_steps = r_res.status == 0
            ? static_cast<int>(r_res.path.size()) - 1 : 0;

        std::cout << std::left << std::setw(6) << test_id << "| "
                  << std::setw(10) << m_status << "| "
                  << std::fixed << std::setprecision(3)
                  << std::setw(10) << m_ms    << "| "
                  << std::setw(10) << m_steps << "| "
                  << std::setprecision(3)
                  << std::setw(10) << m_pm.tortuosity     << "| "
                  << std::setw(10) << m_pm.avg_turn_deg   << "| "
                  << std::setw(10) << m_pm.near_miss_rate << "| "
                  << std::setw(10) << r_status << "| "
                  << std::setw(10) << r_ms    << "| "
                  << std::setw(10) << r_steps << "| "
                  << std::setw(10) << r_pm.tortuosity     << "| "
                  << std::setw(10) << r_pm.avg_turn_deg   << "| "
                  << std::setw(10) << r_pm.near_miss_rate << "\n";

        ++test_id;
    }

    if (test_id == 1) {
        std::cout << "data/points.txt is empty or has invalid format.\n";
        return;
    }
    std::cout << std::string(150, '-') << "\n";
    PrintStats("MPPI Dynamic",  mppi_stats);
    PrintStats("RMPPI Dynamic", rmppi_stats);
}

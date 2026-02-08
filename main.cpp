#include "Astar.h"
#include "DstarLite.h"
#include "DynamicObstacle.h"
#include "Field.h"
#include "heuristic_functions.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>
std::random_device rd;
std::mt19937 gen(rd());
using Change = std::pair<Coordinates, Coordinates>;
using Changes = std::vector<Change>;

void SaveChangesToFile(const Changes &changes, const std::string &filename) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }

    out << changes.size() << "\n";

    for (const auto &ch : changes) {
        out << ch.first.first << " " << ch.first.second << " "
            << ch.second.first << " " << ch.second.second << "\n";
    }
}

Changes LoadChangesFromFile(const std::string &filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filename);
    }

    int n;
    in >> n;

    if (!in || n < 0) {
        throw std::runtime_error("Invalid file header in: " + filename);
    }

    Changes changes;
    changes.reserve(n);

    for (int i = 0; i < n; i++) {
        Coordinates a, b;
        in >> a.first >> a.second >> b.first >> b.second;

        if (!in) {
            throw std::runtime_error("File ended early: " + filename);
        }

        changes.push_back({a, b});
    }

    return changes;
}

void DrawCurrentStep(Field field,
                     const std::vector<std::pair<int, int>> &traversed_path,
                     std::pair<int, int> current_pos,
                     const std::vector<std::pair<int, int>> &predicted_path,
                     std::pair<int, int> goal_pos) {

    for (size_t i = 0; i < traversed_path.size(); ++i) {
        auto [x, y] = traversed_path[i];
        field.Set(x, y, '@');
    }

    for (auto [x, y] : predicted_path) {
        field.Set(x, y, 'O');
    }

    field.Set(current_pos.first, current_pos.second, 'W');
    field.Set(goal_pos.first, goal_pos.second, 'E');
    field.Draw();
    std::cout << "\n";
}

int TestAstar(Field field, int x1, int y1, int x2, int y2) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    Coordinates current_pos = start_p;

    std::vector<Coordinates> predicted_path;
    std::vector<Coordinates> traversed_path;

    auto result = Astar(current_pos, end_p, field, ManhattanDistance);
    if (!result.has_value()) {
        return -1;
    }

    return 0;
}

int TestDynamicAstar(Field field, int x1, int y1, int x2, int y2) {
    Changes changes = LoadChangesFromFile("changes.txt");

    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    Coordinates current_pos = start_p;

    std::vector<Coordinates> predicted_path;
    std::vector<Coordinates> traversed_path;

    auto result = Astar(current_pos, end_p, field, ManhattanDistance);
    if (!result.has_value()) {
        return -1;
    }
    predicted_path = result->path;

    int step = 0;
    while (current_pos != end_p) {
        Change change = changes[step];
        field.Set(change.first.first, change.first.second, '.');
        field.Set(change.second.first, change.second.second, '#');

        if (predicted_path.empty() || !field.CheckPath(predicted_path)) {
            auto result = Astar(current_pos, end_p, field, ManhattanDistance);
            if (!result.has_value()) {
                return -1;
            }
            predicted_path = result->path;
        }

        traversed_path.push_back(current_pos);
        current_pos = predicted_path.front();
        predicted_path.erase(predicted_path.begin());
        ++step;
    }
    std::cout << "\n" << predicted_path.size() << "\n";
    return 0;
}

int TestDstarLite(Field field, int x1, int y1, int x2, int y2) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    DstarLite dstar_lite = DstarLite(&field, start_p, end_p, ManhattanDistance);
    dstar_lite.ComputeShortestPath();

    Coordinates current_pos = start_p;
    std::vector<Coordinates> predicted_path;
    std::vector<Coordinates> traversed_path;

    while (current_pos != end_p) {

        auto next_node = dstar_lite.GetNextNode();
        if (!next_node.has_value()) {
            return -1;
        }
        traversed_path.push_back(current_pos);
        dstar_lite.MoveStart({next_node->first, next_node->second});
        current_pos = {next_node->first, next_node->second};
    }

    return 0;
}

void GetStaticTest() {
    Field field("map.txt");

    std::ifstream in("points.txt");
    if (!in.is_open()) {
        std::cout << "Cannot open points.txt\n";
        return;
    }

    int test_id = 1;

    int x1, y1, x2, y2;
    while (in >> x1 >> y1 >> x2 >> y2) {

        std::cout << test_id << " ";

        // ---------- A* ----------
        try {
            auto start = std::chrono::high_resolution_clock::now();

            int res = TestAstar(field, x1, y1, x2, y2);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;

            if (res == -1) {
                std::cout << "A*: No path found (" << duration.count()
                          << " ms)\n";
            } else {
                std::cout << "A*: " << duration.count() << " ms ";
            }
        } catch (const std::exception &e) {
            std::cout << "A*: exception: " << e.what() << "\n";
        }

        // ---------- D* Lite ----------
        try {
            auto start = std::chrono::high_resolution_clock::now();

            int res = TestDstarLite(field, x1, y1, x2, y2);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;

            if (res == -1) {
                std::cout << "D* Lite: No path found (" << duration.count()
                          << " ms)\n";
            } else {
                std::cout << "D* Lite: " << duration.count() << " ms\n";
            }
        } catch (const std::exception &e) {
            std::cout << "D* Lite: exception: " << e.what() << "\n";
        }

        // std::cout << "\n";
        test_id++;
    }

    if (test_id == 1) {
        std::cout << "points.txt is empty or has invalid format.\n";
    }
}

void GetDynamicChanges() {
    Field field("map.txt");
    std::uniform_real_distribution<double> dist_width(0, field.GetWidth());
    std::uniform_real_distribution<double> dist_height(0, field.GetHeight());

    int STEPS = 2000;
    int PER_STEP = 1;
    Changes changes;
    for (int i = 0; i < STEPS; i++) {
        Change change{{-1, -1}, {-1, -1}};
        while (change.first.first == -1) {
            int x = dist_width(gen);
            int y = dist_height(gen);
            if (field.IsValid(x, y)) {
                continue;
            }
            if (field.IsValid(x - 1, y)) {
                change = {{x, y}, {x - 1, y}};
            } else if (field.IsValid(x + 1, y)) {
                change = {{x, y}, {x + 1, y}};
            } else if (field.IsValid(x, y - 1)) {
                change = {{x, y}, {x, y - 1}};
            } else if (field.IsValid(x, y + 1)) {
                change = {{x, y}, {x, y + 1}};
            }
        }
        int x = change.first.first;
        int y = change.first.second;
        int dx = change.second.first;
        int dy = change.second.second;
        field.Set(x, y, '.');
        field.Set(dx, dy, '#');
        changes.push_back(change);
    }
    SaveChangesToFile(changes, "changes.txt");
}

int TestDynamicDstarLite(Field field, int x1, int y1, int x2, int y2) {
    Changes changes = LoadChangesFromFile("changes.txt");

    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    DstarLite dstar_lite = DstarLite(&field, start_p, end_p, ManhattanDistance);
    dstar_lite.ComputeShortestPath();

    Coordinates current_pos = start_p;
    std::vector<Coordinates> predicted_path;
    std::vector<Coordinates> traversed_path;

    int step = 0;
    while (current_pos != end_p) {
        Change change = changes[step];

        dstar_lite.UpdateObstacle(change.first, '.');
        dstar_lite.UpdateObstacle(change.second, '#');

        dstar_lite.ComputeShortestPath();

        auto next_node = dstar_lite.GetNextNode();
        if (!next_node.has_value()) {
            return -1;
        }
        traversed_path.push_back(current_pos);
        dstar_lite.MoveStart({next_node->first, next_node->second});
        current_pos = {next_node->first, next_node->second};
        ++step;
    }

    return 0;
}
void DynamicTests() {
    Field field("map.txt");
    std::cout << field.GetWidth() << " " << field.GetHeight() << "\n";
    return;
    std::uniform_real_distribution<double> dist_width(0, field.GetWidth());
    std::uniform_real_distribution<double> dist_height(0, field.GetHeight());
    int x1 = dist_width(gen);
    int x2 = dist_width(gen);
    int y1 = dist_height(gen);
    int y2 = dist_height(gen);
    auto start = std::chrono::high_resolution_clock::now();
    std::cin >> x1 >> y1 >> x2 >> y2;
    // int res = TestDstarLite(field, x1, y1, x2, y2);
    int res = TestDynamicDstarLite(field, x1, y1, x2, y2);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    if (res == -1) {
        std::cout << "D* Lite: No path found (";
    }
    else {
        std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << "\n";
        std::cout << "D* Lite: " << duration.count() << " ms\n";
    }

}
int main() {
    // Changes changes = LoadChangesFromFile("changes.txt");
    // GetDynamicChanges();
    // GetStaticTest();
    DynamicTests();
    return 0;
}

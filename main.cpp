#include "Astar.h"
#include "DstarLite.h"
#include "DynamicObstacle.h"
#include "Field.h"
#include "heuristic_functions.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
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

void TestAstar() {
    auto start = std::chrono::high_resolution_clock::now();
    Field field("map.txt");
    DynamicObstacle obstacle = DynamicObstacle(&field, {13, 5});

    constexpr Coordinates start_p = {9, 0};
    constexpr Coordinates end_p = {11, 20};
    Coordinates current_pos = start_p;

    std::vector<Coordinates> predicted_path;
    std::vector<Coordinates> traversed_path;

    while (current_pos != end_p) {
        obstacle.MakeStep();
        obstacle.ReservePosition(
                {obstacle.current_pos.first - 1, obstacle.current_pos.second});
        if (predicted_path.empty() || !field.CheckPath(predicted_path)) {
            auto result = Astar(current_pos, end_p, field, ManhattanDistance);
            if (!result.has_value()) {
                std::cout << "No path found" << std::endl;
                return;
            }
            predicted_path = result->path;
        }

        traversed_path.push_back(current_pos);
        current_pos = predicted_path.front();
        predicted_path.erase(predicted_path.begin());
        // DrawCurrentStep(field, traversed_path, current_pos, predicted_path,
        // end_p);
    }

    std::cout << "Goal!";
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Время выполнения: " << duration.count() << " мс" << std::endl;
}

void TestDstarLite() {
    Coordinates obstacle_pos = {14, 5};
    auto start = std::chrono::high_resolution_clock::now();
    Field field("map.txt");

    constexpr std::pair<int, int> start_p = {9, 0};
    constexpr std::pair<int, int> end_p = {13, 20};
    DstarLite dstar_lite = DstarLite(&field, start_p, end_p, ManhattanDistance);
    dstar_lite.ComputeShortestPath();

    Coordinates current_pos = start_p;
    std::vector<Coordinates> predicted_path;
    std::vector<Coordinates> traversed_path;

    while (current_pos != end_p) {
        if (obstacle_pos.first >= 0) {
            field.Set(obstacle_pos.first, obstacle_pos.second, '.');
            dstar_lite.UpdateObstacle(obstacle_pos, '.');
            obstacle_pos.first --;
        }
        if (field.IsValid(obstacle_pos.first, obstacle_pos.second)) {
            field.Set(obstacle_pos.first, obstacle_pos.second, 'H');
            dstar_lite.UpdateObstacle(obstacle_pos, 'H');
        }
        if (field.IsValid(obstacle_pos.first-1, obstacle_pos.second)) {
            field.Set(obstacle_pos.first-1, obstacle_pos.second, 'H');
            dstar_lite.UpdateObstacle({obstacle_pos.first - 1, obstacle_pos.second}, 'H');
        }
        dstar_lite.ComputeShortestPath();

        auto next_node = dstar_lite.GetNextNode();
        if (!next_node.has_value()) {
            std::cout << "No path found" << std::endl;
            return;
        }
        traversed_path.push_back(current_pos);
        dstar_lite.MoveStart({next_node->first, next_node->second});
        current_pos = {next_node->first, next_node->second};
        // DrawCurrentStep(field, traversed_path, current_pos, predicted_path,
        // end_p);
    }

    std::cout << "Goal!";
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Время выполнения: " << duration.count() << " мс" << std::endl;
}
int main() {
    // TestAstar();
    TestDstarLite();

    return 0;
}

#pragma once
#include <optional>
#include <utility>
#include <vector>
#include "Field.h"

#include "set"
#include <functional>
#include <map>


class AstarNode {
public:
    int x;
    int y;
    float g;
    float h;
    float f;
    AstarNode *parent;

    AstarNode() ;

    AstarNode(const int x, const int y) : x(x), y(y), g(0), h(0), f(0), parent() {}

    AstarNode(const int x, const int y, const float g, const float h, AstarNode *parent)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

    AstarNode(const int x, const int y, const float g, const float h)
        : x(x), y(y), g(g), h(h), f(g + h), parent(nullptr) {}

    bool operator<(const AstarNode &other) const ;
};


struct Result {
    std::vector<std::pair<int,int>> path;
    size_t iterations_count;
    size_t search_tree_size;
};

std::optional<Result> Astar(const std::pair<int, int> &start_pos, const std::pair<int, int> &goal_pos,
      Field &field,
      const std::function<int(std::pair<int, int>, std::pair<int, int>)>
              &heuristic_function);
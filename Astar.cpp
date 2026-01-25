#include "Astar.h"

#include <algorithm>


AstarNode::AstarNode() {
    x = -1;
    y = -1;
    g = -1;
    h = -1;
    f = -1;
    parent = nullptr;
}

bool AstarNode::operator<(const AstarNode &other) const {
    return std::make_tuple(f, g, x, y) <
           std::make_tuple(other.f, other.g, other.x, other.y);
}

std::optional<Result>
Astar(const std::pair<int, int> &start_pos, const std::pair<int, int> &goal_pos,
      Field &field,
      const std::function<int(std::pair<int, int>, std::pair<int, int>)>
              &heuristic_function) {

    std::set<AstarNode> open;
    std::map<std::pair<int, int>, AstarNode> visited;

    const AstarNode start(start_pos.first, start_pos.second, 0,
                     heuristic_function(start_pos, goal_pos));

    visited[start_pos] = start;
    open.insert(visited[start_pos]);

    size_t steps = 0;
    while (!open.empty()) {
        AstarNode current_val = *open.begin();
        open.erase(open.begin());

        AstarNode *current_ptr = &visited[{current_val.x, current_val.y}];

        if (current_ptr->x == goal_pos.first &&
            current_ptr->y == goal_pos.second) {
            std::vector<std::pair<int, int>> path;
            AstarNode *path_ptr = current_ptr;

            while (path_ptr->parent) {
                path.emplace_back(path_ptr->x, path_ptr->y);
                path_ptr = path_ptr->parent;
            }

            const size_t tree_size = visited.size();
            std::reverse(path.begin(), path.end());

            return Result(path, steps, tree_size);
        }

        for (auto [i, j] :
             field.GetNeighbours(current_ptr->x, current_ptr->y)) {

            const float new_g = current_ptr->g + 1;
            const std::pair<int, int> next_pos = {i, j};

            if (visited.find(next_pos) == visited.end() ||
                new_g < visited[next_pos].g) {
                if (visited.count(next_pos)) {
                    open.erase(visited[next_pos]);
                }

                AstarNode next_node(i, j, new_g,
                               heuristic_function({i, j}, goal_pos),
                               current_ptr);
                visited[next_pos] = next_node;
                open.insert(visited[next_pos]);
            }
        }
        steps++;
    }

    return std::nullopt;
}
#include "heuristic_functions.h"
#include <cmath>

// int ManhattanDistance(const int x1, const int y1, const int x2, const int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

int ManhattanDistance(const std::pair<int, int> &a,
                      const std::pair<int, int> &b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}
#pragma once

#include <utility>
#include "Field.h"

class DynamicObstacle {
    public:
    std::pair<int, int> current_pos;
    std::pair<int, int> reserved_pos;
    Field* field_ptr;

    DynamicObstacle(Field* field_ptr, const std::pair<int, int> &current_pos);

    void ReservePosition(const std::pair<int, int> &position);
    void MakeStep();

};



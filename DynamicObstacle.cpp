#include "DynamicObstacle.h"

DynamicObstacle::DynamicObstacle(Field *field_ptr,
                                 const std::pair<int, int> &current_pos)
    : field_ptr(field_ptr), current_pos(current_pos) {
    reserved_pos = current_pos;
    field_ptr->Set(current_pos.first, current_pos.second, 'D');
}

void DynamicObstacle::ReservePosition(const std::pair<int, int> &position) {
    if (position == current_pos) {
        return;
    }
    if (field_ptr->IsValid(position.first, position.second)) {
        reserved_pos = position;
        field_ptr->Set(reserved_pos.first, reserved_pos.second, 'R');
    }
}

void DynamicObstacle::MakeStep() {
    if (reserved_pos == current_pos) {
        return;
    }
    field_ptr->Set(current_pos.first, current_pos.second, '.');
    current_pos = reserved_pos;
    field_ptr->Set(reserved_pos.first, reserved_pos.second, 'D');
}
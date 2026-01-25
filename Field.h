#pragma once
#include <string>
#include <vector>
#include <iostream>

class Field {
  public:
    explicit Field(const std::string &file_name);

     [[nodiscard]] char Get(size_t x, size_t y) const;

    void Set(size_t x, size_t y, char value);

    [[nodiscard]] bool IsValid(int x, int y) const ;

    [[nodiscard]] std::vector<std::pair<int, int>>
    GetNeighbours(int x,  int y) const ;

    void Draw() const;

    [[nodiscard]] bool CheckPath(const std::vector<std::pair<int, int>>& path) const;

    size_t GetWidth() const;
    size_t GetHeight() const;
  private:
    size_t width;
    size_t height;
    std::vector<std::vector<char>> field_data_;
};
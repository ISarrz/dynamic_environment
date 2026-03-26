#pragma once

#include <filesystem>
#include <string>

inline std::string GetDataDirPath() {
    // Running from build/ should still use the project-level data directory.
    if (std::filesystem::exists("../CMakeLists.txt")) {
        return "../data";
    }
    return "data";
}

inline std::string GetMapPath() { return GetDataDirPath() + "/map.txt"; }
inline std::string GetPointsPath() { return GetDataDirPath() + "/points.txt"; }
inline std::string GetChangesPath() { return GetDataDirPath() + "/changes.txt"; }

inline void EnsureDataDir() {
    std::filesystem::create_directories(GetDataDirPath());
}

#include "CliUtils.h"
#include "DynamicTests.h"
#include "Generators.h"
#include "StaticTests.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
int ParseNonNegativeInt(const std::string &value, const std::string &arg_name) {
    int parsed = 0;
    try {
        parsed = std::stoi(value);
    } catch (...) {
        throw std::runtime_error("Invalid " + arg_name + ": " + value);
    }
    if (parsed < 0) {
        throw std::runtime_error(arg_name + " must be non-negative");
    }
    return parsed;
}
}

int main(int argc, char **argv) {
    // Changes changes = LoadChangesFromFile("changes.txt");
    // GetDynamicChanges(2000, 10);
    // GetStaticTest();
    const int default_objects_per_step = 10;
    try {
        // Modes:
        //   ./dynamic_environment generate [steps] [objects_per_step]
        //                                  -> only generate changes.txt
        //   ./dynamic_environment generate-points [count]
        //                                  -> generate points.txt
        //   ./dynamic_environment generate-points [count] [min_distance]
        //                                  -> only generate points.txt
        //   ./dynamic_environment test dynamic -> run dynamic test using existing changes.txt
        //   ./dynamic_environment test static  -> run static tests from points.txt
        if (argc > 1) {
            std::string mode = argv[1];
            if (mode == "generate") {
                int steps = 2000;
                int objects_per_step = default_objects_per_step;

                if (argc >= 3) {
                    steps = ParsePositiveInt(argv[2], "steps");
                }
                if (argc >= 4) {
                    objects_per_step =
                        ParsePositiveInt(argv[3], "objects_per_step");
                }
                if (argc > 4) {
                    std::cerr << "Too many arguments for generate mode\n";
                    return 1;
                }

                GenerateDynamicChangesOnly(objects_per_step, steps);
                return 0;
            }
            if (mode == "generate-points") {
                int points_count = 100;
                int min_distance = -1;
                if (argc >= 3) {
                    points_count = ParsePositiveInt(argv[2], "points_count");
                }
                if (argc >= 4) {
                    min_distance =
                        ParseNonNegativeInt(argv[3], "min_distance");
                }
                if (argc > 4) {
                    std::cerr << "Too many arguments for generate-points mode\n";
                    return 1;
                }
                GeneratePointsOnly(points_count, min_distance);
                return 0;
            }
            if (mode == "test") {
                if (argc != 3) {
                    std::cerr << "Usage: ./dynamic_environment test [dynamic|static]\n";
                    return 1;
                }
                std::string test_mode = argv[2];
                if (test_mode == "dynamic") {
                    DynamicTests();
                    return 0;
                }
                if (test_mode == "static") {
                    GetStaticTest();
                    return 0;
                }
                std::cerr << "Unknown test mode: " << test_mode << "\n"
                          << "Usage: ./dynamic_environment test [dynamic|static]\n";
                return 1;
            }
            std::cerr << "Unknown mode: " << mode << "\n"
                      << "Usage: ./dynamic_environment [generate|generate-points|test]\n";
            return 1;
        }

        std::cout << "Usage:\n";
        std::cout << "  ./dynamic_environment generate [steps] [objects_per_step]\n";
        std::cout
            << "  ./dynamic_environment generate-points [count] [min_distance]\n";
        std::cout << "  ./dynamic_environment test [dynamic|static]\n";
        std::cout << "Run 'generate' and 'generate-points' before tests.\n";
    } catch (const std::exception &e) {
        std::cerr << "Runtime error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

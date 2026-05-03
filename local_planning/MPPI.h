#pragma once
#include "Field.h"
#include <optional>
#include <random>
#include <utility>
#include <vector>

struct MPPIConfig {
    int   num_samples     = 50;
    int   horizon         = 12;
    float lambda          = 1.5f;
    float sigma           = 0.8f;
    float obstacle_cost   = 200.0f;
    float step_weight     = 0.1f;
    float terminal_weight = 15.0f;
};

class MPPI {
public:
    explicit MPPI(MPPIConfig config, unsigned int seed = 42);
    virtual ~MPPI() = default;

    // Returns next grid cell to move to, or nullopt if stuck.
    std::optional<std::pair<int, int>> GetNextNode(
        std::pair<int, int> current,
        std::pair<int, int> goal,
        const Field& field);

protected:
    // Can be overridden by subclasses (e.g. RMPPI uses CVaR filtering).
    virtual std::vector<float> ComputeWeights(
        const std::vector<float>& costs) const;

    MPPIConfig cfg_;
    std::mt19937 rng_;
    mutable std::vector<float> noise_buf_; // pre-allocated noise buffer
};

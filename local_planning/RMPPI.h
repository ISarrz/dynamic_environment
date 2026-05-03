#pragma once
#include "MPPI.h"

// RMPPI extends MPPI with CVaR-based weight filtering:
// only the best alpha-fraction of samples (lowest cost) contribute to the
// control update. This makes the planner robust to worst-case trajectories.
struct RMPPIConfig : MPPIConfig {
    float cvar_alpha = 0.4f; // fraction of best samples to keep (0 < alpha <= 1)
};

class RMPPI : public MPPI {
public:
    explicit RMPPI(RMPPIConfig config, unsigned int seed = 42);

protected:
    std::vector<float> ComputeWeights(
        const std::vector<float>& costs) const override;

private:
    float cvar_alpha_;
};

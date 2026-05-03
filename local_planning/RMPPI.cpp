#include "RMPPI.h"

#include <algorithm>
#include <cmath>
#include <numeric>

RMPPI::RMPPI(RMPPIConfig config, unsigned int seed)
    : MPPI(config, seed), cvar_alpha_(config.cvar_alpha) {}

std::vector<float> RMPPI::ComputeWeights(const std::vector<float>& costs) const {
    int N    = static_cast<int>(costs.size());
    int keep = std::max(1, static_cast<int>(std::round(N * cvar_alpha_)));

    // Sort indices by cost ascending; use only the `keep` best.
    std::vector<int> idx(N);
    std::iota(idx.begin(), idx.end(), 0);
    std::partial_sort(idx.begin(), idx.begin() + keep, idx.end(),
        [&costs](int a, int b) { return costs[a] < costs[b]; });

    std::vector<float> weights(N, 0.0f);
    float min_cost = costs[idx[0]];
    float sum      = 0.0f;

    for (int i = 0; i < keep; ++i) {
        int k      = idx[i];
        weights[k] = std::exp(-(costs[k] - min_cost) / cfg_.lambda);
        sum       += weights[k];
    }
    if (sum > 0.0f)
        for (auto& w : weights) w /= sum;
    return weights;
}

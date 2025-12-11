#include "pivot_planner/pivot_sampler.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace pivot_planner {

PivotSampler::PivotSampler(Sampler& base_sampler,
                           FeatureExtractor& feat_extractor,
                           const Theta& theta,
                           double alpha,
                           double tau,
                           int batch_size,
                           unsigned int seed)
    : base_sampler_(base_sampler),
      feat_extractor_(feat_extractor),
      theta_(theta),
      alpha_(alpha),
      tau_(tau),
      batch_size_(batch_size),
      rng_(seed),
      dist_01_(0.0, 1.0)
{
}

State PivotSampler::sample() {
    double r = dist_01_(rng_);
    
    if (r > alpha_) {
        // (1 - α) branch: base sampler p_0
        return base_sampler_.sample();
    }
    
    // α branch: prompt-tilted sampler p_τ
    return sampleTiltedBatch();
}

State PivotSampler::sampleTiltedBatch() {
    std::vector<State> candidates(batch_size_);
    std::vector<double> utilities(batch_size_);
    std::vector<double> weights(batch_size_);
    
    //  Sample candidates from base p_0
    for (int i = 0; i < batch_size_; ++i) {
        candidates[i] = base_sampler_.sample();
        Features f = feat_extractor_.compute(candidates[i]);
        utilities[i] = utility(f, theta_);
    }
    
    //  Compute softmax weights ∝ exp(τ U_θ(x))
    double max_u = *std::max_element(utilities.begin(), utilities.end());
    double sum_w = 0.0;
    
    for (int i = 0; i < batch_size_; ++i) {
        // Subtract max for numerical stability
        weights[i] = std::exp(tau_ * (utilities[i] - max_u));
        sum_w += weights[i];
    }
    
    // Normalize
    for (int i = 0; i < batch_size_; ++i) {
        weights[i] /= sum_w;
    }
    
    // Sample index according to weights
    int idx = sampleIndexFromDistribution(weights);
    return candidates[idx];
}

int PivotSampler::sampleIndexFromDistribution(const std::vector<double>& weights) {
    double r = dist_01_(rng_);
    double cumsum = 0.0;
    
    for (size_t i = 0; i < weights.size(); ++i) {
        cumsum += weights[i];
        if (r <= cumsum) {
            return static_cast<int>(i);
        }
    }
    
    // Fallback (should rarely happen due to numerical precision)
    return static_cast<int>(weights.size() - 1);
}

} // namespace pivot_planner

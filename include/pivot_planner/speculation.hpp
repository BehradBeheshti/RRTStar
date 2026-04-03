#ifndef PIVOT_PLANNER_SPECULATION_HPP
#define PIVOT_PLANNER_SPECULATION_HPP

#include "pivot_planner/types.hpp"
#include "pivot_planner/sampler.hpp"
#include "pivot_planner/feature_extractor.hpp"

namespace pivot_planner {

// Context for speculative evaluation
struct SpeculationContext {
    Sampler* sampler;                    // Sampler for speculative rollout
    const FeatureExtractor* feat;        // Feature extractor
    const Theta* theta;                  // Weight vector
    int K;                               // Rollout length (0 = disabled)
    double min_future_utility;           // Acceptance threshold
    double step_size;                    // Step size for rollout
    
    SpeculationContext()
        : sampler(nullptr), feat(nullptr), theta(nullptr),
          K(0), min_future_utility(0.3), step_size(0.2) {}
};

// Speculative acceptance function
// Performs a short rollout from x_new and checks if future utility is promising
bool speculativeAccept(const State& x_new, const SpeculationContext& ctx);

// Helper: steer from x_from towards x_to with max step step_size
State steerTowards(const State& x_from, const State& x_to, double step_size);

// Simple collision checker (returns true if path is collision-free)
// For now, just checks bounds. Extend this for real obstacle checking.
bool isCollisionFree(const State& x_from, const State& x_to,
                     double xmin, double xmax, double ymin, double ymax);

} // namespace pivot_planner

#endif // PIVOT_PLANNER_SPECULATION_HPP

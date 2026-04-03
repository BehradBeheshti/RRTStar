#ifndef PIVOT_PLANNER_PIVOT_SAMPLER_HPP
#define PIVOT_PLANNER_PIVOT_SAMPLER_HPP

#include "pivot_planner/sampler.hpp"
#include "pivot_planner/feature_extractor.hpp"
#include "pivot_planner/types.hpp"
#include <vector>

namespace pivot_planner {

// PIVOT sampler: mixture of base sampler and exponentially-tilted sampler
// p_{α,τ}(x) = (1-α)p_0(x) + α p_τ(x)
class PivotSampler : public Sampler {
public:
    PivotSampler(Sampler& base_sampler,
                 FeatureExtractor& feat_extractor,
                 const Theta& theta,
                 double alpha = 0.5,
                 double tau = 1.0,
                 int batch_size = 20,
                 unsigned int seed = std::random_device{}());
    
    State sample() override;
    
    // Parameter setters
    void setTheta(const Theta& theta) { theta_ = theta; }
    void setAlpha(double alpha) { alpha_ = alpha; }
    void setTau(double tau) { tau_ = tau; }
    void setBatchSize(int batch_size) { batch_size_ = batch_size; }
    
    // Getters
    double getAlpha() const { return alpha_; }
    double getTau() const { return tau_; }
    const Theta& getTheta() const { return theta_; }
    
private:
    Sampler& base_sampler_;
    FeatureExtractor& feat_extractor_;
    Theta theta_;
    double alpha_;  // Mixture parameter ∈ (0,1)
    double tau_;    // Temperature parameter > 0
    int batch_size_;
    
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_01_;
    
    // Sample from the tilted distribution p_τ using batch softmax
    State sampleTiltedBatch();
    
    // Sample index from discrete distribution
    int sampleIndexFromDistribution(const std::vector<double>& weights);
};

} // namespace pivot_planner

#endif // PIVOT_PLANNER_PIVOT_SAMPLER_HPP

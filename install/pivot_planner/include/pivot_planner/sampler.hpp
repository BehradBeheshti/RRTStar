#ifndef PIVOT_PLANNER_SAMPLER_HPP
#define PIVOT_PLANNER_SAMPLER_HPP

#include "pivot_planner/types.hpp"
#include <random>

namespace pivot_planner {

// Abstract sampler interface
class Sampler {
public:
    virtual ~Sampler() = default;
    virtual State sample() = 0;
};

// Uniform sampler in a bounded region (base sampler p_0)
class UniformSampler : public Sampler {
public:
    UniformSampler(double xmin, double xmax, double ymin, double ymax, 
                   unsigned int seed = std::random_device{}());
    
    State sample() override;
    
    void setBounds(double xmin, double xmax, double ymin, double ymax);
    
private:
    double xmin_, xmax_, ymin_, ymax_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_x_;
    std::uniform_real_distribution<double> dist_y_;
    std::uniform_real_distribution<double> dist_yaw_;
    
    void updateDistributions();
};

// Goal-biased sampler (optional enhancement to base sampler)
class GoalBiasedSampler : public Sampler {
public:
    GoalBiasedSampler(Sampler& base_sampler, const State& goal, 
                      double goal_bias = 0.05,
                      unsigned int seed = std::random_device{}());
    
    State sample() override;
    
    void setGoal(const State& goal) { goal_ = goal; }
    void setGoalBias(double bias) { goal_bias_ = bias; }
    
private:
    Sampler& base_sampler_;
    State goal_;
    double goal_bias_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_01_;
};

} // namespace pivot_planner

#endif // PIVOT_PLANNER_SAMPLER_HPP

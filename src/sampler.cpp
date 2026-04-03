#include "pivot_planner/sampler.hpp"
#include <cmath>

namespace pivot_planner {

// UniformSampler implementation
UniformSampler::UniformSampler(double xmin, double xmax, double ymin, double ymax, 
                               unsigned int seed)
    : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax),
      rng_(seed),
      dist_yaw_(-M_PI, M_PI)
{
    updateDistributions();
}

State UniformSampler::sample() {
    State s;
    s.x = dist_x_(rng_);
    s.y = dist_y_(rng_);
    s.yaw = dist_yaw_(rng_);
    return s;
}

void UniformSampler::setBounds(double xmin, double xmax, double ymin, double ymax) {
    xmin_ = xmin;
    xmax_ = xmax;
    ymin_ = ymin;
    ymax_ = ymax;
    updateDistributions();
}

void UniformSampler::updateDistributions() {
    dist_x_ = std::uniform_real_distribution<double>(xmin_, xmax_);
    dist_y_ = std::uniform_real_distribution<double>(ymin_, ymax_);
}

// GoalBiasedSampler implementation
GoalBiasedSampler::GoalBiasedSampler(Sampler& base_sampler, const State& goal, 
                                     double goal_bias, unsigned int seed)
    : base_sampler_(base_sampler), goal_(goal), goal_bias_(goal_bias),
      rng_(seed), dist_01_(0.0, 1.0)
{
}

State GoalBiasedSampler::sample() {
    double r = dist_01_(rng_);
    if (r < goal_bias_) {
        // Return goal with probability goal_bias
        return goal_;
    } else {
        // Sample from base sampler
        return base_sampler_.sample();
    }
}

} // namespace pivot_planner

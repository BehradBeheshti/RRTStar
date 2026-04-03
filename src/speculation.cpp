#include "pivot_planner/speculation.hpp"
#include <cmath>

namespace pivot_planner {

bool speculativeAccept(const State& x_new, const SpeculationContext& ctx) {
    // If speculation is disabled (K = 0), always accept
    if (ctx.K <= 0) {
        return true;
    }
    
    // Check for null pointers
    if (!ctx.sampler || !ctx.feat || !ctx.theta) {
        return true;  // Fallback: accept if context is invalid
    }
    
    double sum_utility = 0.0;
    State x = x_new;
    int valid_steps = 0;
    
    for (int k = 0; k < ctx.K; ++k) {
        // Sample a target direction
        State target = ctx.sampler->sample();
        
        // Steer towards it
        State x_next = steerTowards(x, target, ctx.step_size);
        
        // Check collision (simple bounds check for now)
        // TODO: Replace with actual collision checking
        if (x_next.x < -10.0 || x_next.x > 10.0 || 
            x_next.y < -10.0 || x_next.y > 10.0) {
            // Out of bounds - treat as bad utility
            Features f_bad;
            f_bad.goal_progress = 0.0;
            f_bad.safety = 0.0;
            f_bad.exploration = 0.0;
            sum_utility += utility(f_bad, *ctx.theta);
            break;
        }
        
        // Compute utility for this step
        Features f = ctx.feat->compute(x_next);
        double u = utility(f, *ctx.theta);
        sum_utility += u;
        
        x = x_next;
        valid_steps++;
    }
    
    // Compute average utility
    double avg_utility = (valid_steps > 0) ? (sum_utility / valid_steps) : 0.0;
    
    // Accept if average utility meets threshold
    return (avg_utility >= ctx.min_future_utility);
}

State steerTowards(const State& x_from, const State& x_to, double step_size) {
    double dx = x_to.x - x_from.x;
    double dy = x_to.y - x_from.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    if (dist < 1e-6) {
        return x_from;  // Already there
    }
    
    // Clamp to step size
    double t = std::min(1.0, step_size / dist);
    
    State result;
    result.x = x_from.x + t * dx;
    result.y = x_from.y + t * dy;
    
    // Interpolate yaw (simple linear interpolation)
    result.yaw = x_from.yaw + t * (x_to.yaw - x_from.yaw);
    
    // Normalize yaw to [-π, π]
    while (result.yaw > M_PI) result.yaw -= 2.0 * M_PI;
    while (result.yaw < -M_PI) result.yaw += 2.0 * M_PI;
    
    return result;
}

bool isCollisionFree(const State& x_from, const State& x_to,
                     double xmin, double xmax, double ymin, double ymax) {
    // Simple bounds check
    // TODO: Replace with actual collision checking using occupancy grid
    
    if (x_from.x < xmin || x_from.x > xmax || 
        x_from.y < ymin || x_from.y > ymax) {
        return false;
    }
    
    if (x_to.x < xmin || x_to.x > xmax || 
        x_to.y < ymin || x_to.y > ymax) {
        return false;
    }
    
    // For now, just check endpoints. Ideally, check along the path.
    return true;
}

} // namespace pivot_planner

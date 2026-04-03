#ifndef PIVOT_PLANNER_TYPES_HPP
#define PIVOT_PLANNER_TYPES_HPP

#include <vector>
#include <memory>
#include <cmath>

namespace pivot_planner {

// Robot state in configuration space (for 2D UGV: x, y, yaw)
struct State {
    double x;
    double y;
    double yaw;
    
    State() : x(0.0), y(0.0), yaw(0.0) {}
    State(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
    
    // Distance metric (Euclidean for now, can extend to Dubins/Reeds-Shepp)
    double distance(const State& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

// One node in the RRT* tree
struct Node {
    State state;
    Node* parent;
    double cost_from_start;  // Geometric cost (path length)
    
    Node(const State& s, Node* p = nullptr, double cost = 0.0)
        : state(s), parent(p), cost_from_start(cost) {}
};

// Features φ(x) in [0,1]^m
struct Features {
    double goal_progress;   // [0,1]: higher = closer to goal
    double safety;          // [0,1]: higher = safer (far from obstacles)
    double exploration;     // [0,1]: higher = less visited region
    
    Features() : goal_progress(0.0), safety(1.0), exploration(0.5) {}
};

// Prompt weights θ
struct Theta {
    double w_goal;
    double w_safety;
    double w_exploration;
    
    Theta() : w_goal(1.0), w_safety(1.0), w_exploration(0.5) {}
    Theta(double g, double s, double e) : w_goal(g), w_safety(s), w_exploration(e) {}
};

// Utility function: U_θ(x) = θ^T φ(x)
inline double utility(const Features& f, const Theta& theta) {
    return theta.w_goal * f.goal_progress
         + theta.w_safety * f.safety
         + theta.w_exploration * f.exploration;
}

} // namespace pivot_planner

#endif // PIVOT_PLANNER_TYPES_HPP

#ifndef PIVOT_PLANNER_PIVOT_RRT_STAR_HPP
#define PIVOT_PLANNER_PIVOT_RRT_STAR_HPP

#include "pivot_planner/types.hpp"
#include "pivot_planner/pivot_sampler.hpp"
#include "pivot_planner/feature_extractor.hpp"
#include "pivot_planner/speculation.hpp"
#include "pivot_planner/map_collision_checker.hpp"
#include <vector>
#include <memory>

namespace pivot_planner {

// PIVOT-RRT* Planner
class PivotRRTStar {
public:
    PivotRRTStar(PivotSampler& sampler,
                 FeatureExtractor& feat_extractor,
                 const Theta& theta,
                 const State& start,
                 const State& goal,
                 double xmin, double xmax, double ymin, double ymax,
                 double step_size = 0.5,
                 double neighbor_radius = 1.0,
                 double goal_tolerance = 0.3,
                 std::shared_ptr<MapCollisionChecker> collision_checker = nullptr);
    
    ~PivotRRTStar();
    
    // Run planning for max_iterations
    void run(int max_iterations);
    
    // Single planning step
    void step();
    
    // Get best path found so far
    std::vector<State> getBestPath() const;
    
    // Check if goal has been reached
    bool hasReachedGoal() const;
    
    // Get tree for visualization
    const std::vector<Node*>& getTree() const { return tree_; }
    
    // Get/Set speculation context
    void setSpeculationContext(const SpeculationContext& ctx) { spec_ctx_ = ctx; }
    SpeculationContext& getSpeculationContext() { return spec_ctx_; }
    
    // Statistics
    int getIterationCount() const { return iteration_count_; }
    int getTreeSize() const { return tree_.size(); }
    
private:
    PivotSampler& sampler_;
    FeatureExtractor& feat_extractor_;
    Theta theta_;
    State start_;
    State goal_;
    
    double xmin_, xmax_, ymin_, ymax_;
    double step_size_;
    double neighbor_radius_;
    double goal_tolerance_;
    std::shared_ptr<MapCollisionChecker> collision_checker_;
    
    SpeculationContext spec_ctx_;
    
    std::vector<Node*> tree_;
    Node* best_goal_node_;
    int iteration_count_;
    
    // RRT* helpers
    Node* nearestNode(const State& s) const;
    std::vector<Node*> findNeighbors(const State& s, double radius) const;
    
    // Check if state is within bounds
    bool isValid(const State& s) const;
};

} // namespace pivot_planner

#endif // PIVOT_PLANNER_PIVOT_RRT_STAR_HPP

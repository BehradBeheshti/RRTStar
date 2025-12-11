#ifndef PIVOT_PLANNER_PIVOT_RRT_STAR_HPP
#define PIVOT_PLANNER_PIVOT_RRT_STAR_HPP

#include "pivot_planner/types.hpp"
#include "pivot_planner/pivot_sampler.hpp"
#include "pivot_planner/feature_extractor.hpp"
#include "pivot_planner/speculation.hpp"
#include "pivot_planner/map_collision_checker.hpp"
#include <vector>
#include <memory>
#include <unordered_map>
#include <queue>
#include <stack>

namespace pivot_planner {

// Spatial hash for fast neighbor lookup
struct GridCell {
    int x, y;
    
    GridCell(int x_, int y_) : x(x_), y(y_) {}
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct GridCellHash {
    size_t operator()(const GridCell& cell) const {
        // Cantor pairing function for hashing
        return std::hash<int>()((cell.x + cell.y) * (cell.x + cell.y + 1) / 2 + cell.y);
    }
};

// Node comparator for priority queue (lower cost = higher priority)
struct NodeComparator {
    bool operator()(const Node* a, const Node* b) const {
        return a->cost_from_start > b->cost_from_start;  // Min-heap
    }
};

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
    
    // Get all nodes sorted by cost (SORTING ALGORITHM)
    std::vector<Node*> getSortedNodesByCost() const;
    
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
    
    // Spatial hash table for O(1) average neighbor lookup
    std::unordered_map<GridCell, std::vector<Node*>, GridCellHash> spatial_hash_;
    double grid_cell_size_;
    
    // Priority queue for best-first node expansion
    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> expansion_queue_;
    
    // Stack for DFS-style path exploration and backtracking
    std::stack<Node*> exploration_stack_;
    
    // RRT* helpers
    Node* nearestNode(const State& s) const;
    std::vector<Node*> findNeighbors(const State& s, double radius) const;
    
    // Check if state is within bounds
    bool isValid(const State& s) const;
    
    // Helper functions for spatial hash
    GridCell getGridCell(const State& s) const;
    void insertIntoHash(Node* node);
    std::vector<Node*> getNodesInCell(const GridCell& cell) const;
    
    // Helper functions for priority queue
    void addToQueue(Node* node);
    Node* getNextBestNode();
    bool queueEmpty() const;
    
    // Helper functions for stack-based DFS exploration
    void pushToStack(Node* node);
    Node* popFromStack();
    bool stackEmpty() const;
};

} // namespace pivot_planner

#endif // PIVOT_PLANNER_PIVOT_RRT_STAR_HPP
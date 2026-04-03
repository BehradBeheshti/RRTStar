#ifndef PIVOT_PLANNER_FEATURE_EXTRACTOR_HPP
#define PIVOT_PLANNER_FEATURE_EXTRACTOR_HPP

#include "pivot_planner/types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <unordered_map>

namespace pivot_planner {

class VisitGrid {
public:
    VisitGrid(double xmin, double xmax, double ymin, double ymax, double cell_size);
    void markVisited(double x, double y);
    int getVisitCount(double x, double y) const;
    
private:
    double xmin_, xmax_, ymin_, ymax_;
    double cell_size_;
    int nx_, ny_;
    std::vector<int> grid_;
    int getIndex(double x, double y) const;
};

class FeatureExtractor {
public:
    FeatureExtractor(const State& start, const State& goal,
                     double xmin, double xmax, double ymin, double ymax,
                     double safety_margin = 0.8);
    
    Features compute(const State& x) const;
    void markVisited(const State& x);
    
    // NEW: Set occupancy grid for real obstacle distance
    void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    
    double getObstacleDistance(const State& x) const;
    
    void setGoal(const State& goal) { goal_ = goal; }
    const State& getGoal() const { return goal_; }
    
private:
    State start_;
    State goal_;
    double xmin_, xmax_, ymin_, ymax_;
    double safety_margin_;
    double max_dist_;
    
    VisitGrid visit_grid_;
    
    // Occupancy grid for safety feature
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
    bool has_occupancy_grid_;
    
    double computeGoalProgress(const State& x) const;
    double computeSafety(const State& x) const;
    double computeExploration(const State& x) const;
    
    // NEW: Compute distance to nearest obstacle in occupancy grid
    double computeObstacleDistance(const State& x) const;
};

} // namespace pivot_planner

#endif

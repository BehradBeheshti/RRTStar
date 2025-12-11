#include "pivot_planner/feature_extractor.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace pivot_planner {

// VisitGrid implementation
VisitGrid::VisitGrid(double xmin, double xmax, double ymin, double ymax, double cell_size)
    : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax), cell_size_(cell_size)
{
    nx_ = static_cast<int>(std::ceil((xmax_ - xmin_) / cell_size_)) + 1;
    ny_ = static_cast<int>(std::ceil((ymax_ - ymin_) / cell_size_)) + 1;
    grid_.resize(nx_ * ny_, 0);
}

void VisitGrid::markVisited(double x, double y) {
    int idx = getIndex(x, y);
    if (idx >= 0 && idx < static_cast<int>(grid_.size())) {
        grid_[idx]++;
    }
}

int VisitGrid::getVisitCount(double x, double y) const {
    int idx = getIndex(x, y);
    if (idx >= 0 && idx < static_cast<int>(grid_.size())) {
        return grid_[idx];
    }
    return 0;
}

int VisitGrid::getIndex(double x, double y) const {
    int ix = static_cast<int>((x - xmin_) / cell_size_);
    int iy = static_cast<int>((y - ymin_) / cell_size_);
    
    if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_) {
        return -1;
    }
    
    return iy * nx_ + ix;
}

// FeatureExtractor implementation
FeatureExtractor::FeatureExtractor(const State& start, const State& goal,
                                   double xmin, double xmax, double ymin, double ymax,
                                   double safety_margin)
    : start_(start), goal_(goal),
      xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax),
      safety_margin_(safety_margin),
      visit_grid_(xmin, xmax, ymin, ymax, 0.5),
      has_occupancy_grid_(false)
{
    double dx = xmax_ - xmin_;
    double dy = ymax_ - ymin_;
    max_dist_ = std::sqrt(dx*dx + dy*dy);
}

void FeatureExtractor::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    occupancy_grid_ = map;
    has_occupancy_grid_ = true;
}

Features FeatureExtractor::compute(const State& x) const {
    Features f;
    f.goal_progress = computeGoalProgress(x);
    f.safety = computeSafety(x);
    f.exploration = computeExploration(x);
    return f;
}

void FeatureExtractor::markVisited(const State& x) {
    visit_grid_.markVisited(x.x, x.y);
}

double FeatureExtractor::getObstacleDistance(const State& x) const {
    if (has_occupancy_grid_) {
        return computeObstacleDistance(x);
    }
    
    // Fallback: distance to boundaries
    double dist_x_min = x.x - xmin_;
    double dist_x_max = xmax_ - x.x;
    double dist_y_min = x.y - ymin_;
    double dist_y_max = ymax_ - x.y; 
    
    return std::min({dist_x_min, dist_x_max, dist_y_min, dist_y_max});
}

double FeatureExtractor::computeObstacleDistance(const State& x) const {
    if (!has_occupancy_grid_) {
        return 10.0;  // If no map, return large distance (very safe)
    }
    
    int center_ix = (int)((x.x - occupancy_grid_->info.origin.position.x) / occupancy_grid_->info.resolution);
    int center_iy = (int)((x.y - occupancy_grid_->info.origin.position.y) / occupancy_grid_->info.resolution);
    
    //  Search 5 meters around the point to find obstacles
    double max_search_dist = 5.0;  // metersa
    int search_radius = (int)(max_search_dist / occupancy_grid_->info.resolution);
    
    double min_dist = max_search_dist;  // Start with 5.0
    
    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            int ix = center_ix + dx;
            int iy = center_iy + dy;
            
            if (ix < 0 || ix >= (int)occupancy_grid_->info.width ||
                iy < 0 || iy >= (int)occupancy_grid_->info.height) {
                continue;
            }
            
            int idx = iy * occupancy_grid_->info.width + ix;
            if (occupancy_grid_->data[idx] > 50) {  // Occupied
                double wx = occupancy_grid_->info.origin.position.x + ix * occupancy_grid_->info.resolution;
                double wy = occupancy_grid_->info.origin.position.y + iy * occupancy_grid_->info.resolution;
                double dist = std::sqrt((x.x - wx)*(x.x - wx) + (x.y - wy)*(x.y - wy));
                
                // Only count if within search radius
                if (dist <= max_search_dist) {
                    min_dist = std::min(min_dist, dist);
                }
            }
        }
    }
    
    return min_dist;
}

double FeatureExtractor::computeGoalProgress(const State& x) const {
    double dist_start_goal = start_.distance(goal_);
    double dist_x_goal = x.distance(goal_);
    
    if (dist_start_goal < 1e-6) {
        return 1.0;
    }
    
    double progress = 1.0 - (dist_x_goal / dist_start_goal);
    return std::max(0.0, std::min(1.0, progress));
}
double FeatureExtractor::computeSafety(const State& x) const {
    double obs_dist = getObstacleDistance(x);
    
    // Make corridors between obstacles unsafe
  
    if (obs_dist < 1.5) {
        // Anything closer than 1.5m is basically unusable for "safe" paths
        return 0.0;  
    } else if (obs_dist < 2.5) {
        
        return (obs_dist - 1.5) / 1.0; 
    }
    
    // > 2.5m: good safety
    return 1.0;
}
double FeatureExtractor::computeExploration(const State& x) const {
    int visit_count = visit_grid_.getVisitCount(x.x, x.y);
    double exploration = 1.0 / (1.0 + visit_count);
    return exploration;
}

} // namespace pivot_planner
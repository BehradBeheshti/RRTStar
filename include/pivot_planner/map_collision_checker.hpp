#ifndef PIVOT_PLANNER_MAP_COLLISION_CHECKER_HPP
#define PIVOT_PLANNER_MAP_COLLISION_CHECKER_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "pivot_planner/types.hpp"

namespace pivot_planner {

class MapCollisionChecker {
public:
    MapCollisionChecker() : has_map_(false) {}
    
    void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        map_ = map;
        has_map_ = true;
    }
    
    bool isOccupied(double x, double y) const {
        if (!has_map_) return false;
        
        int ix = static_cast<int>((x - map_->info.origin.position.x) / map_->info.resolution);
        int iy = static_cast<int>((y - map_->info.origin.position.y) / map_->info.resolution);
        
        if (ix < 0 || ix >= (int)map_->info.width || 
            iy < 0 || iy >= (int)map_->info.height) {
            return true;  // Out of bounds
        }
        
        int idx = iy * map_->info.width + ix;
        return map_->data[idx] > 50;  // Occupied if > 50%
    }
    
    bool isCollisionFree(const State& from, const State& to) const {
        // Check multiple points along the line
        double dist = from.distance(to);
        int num_checks = std::max(3, (int)(dist / 0.05));  // Check every 5cm
        
        for (int i = 0; i <= num_checks; i++) {
            double t = i / (double)num_checks;
            double x = from.x + t * (to.x - from.x);
            double y = from.y + t * (to.y - from.y);
            
            if (isOccupied(x, y)) {
                return false;
            }
        }
        return true;
    }
    
    bool hasMap() const { return has_map_; }
    nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const { return map_; }
    
private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    bool has_map_;
};

} // namespace pivot_planner

#endif

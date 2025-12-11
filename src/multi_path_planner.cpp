#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>  
#include <queue>   

#include "pivot_planner/pivot_rrt_star.hpp"
#include "pivot_planner/pivot_sampler.hpp"
#include "pivot_planner/feature_extractor.hpp"
#include "pivot_planner/map_collision_checker.hpp"

class MultiPathPlanner : public rclcpp::Node {
public:
    MultiPathPlanner() : Node("multi_path_planner") {
        collision_checker_ = std::make_shared<pivot_planner::MapCollisionChecker>();
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, 
            std::bind(&MultiPathPlanner::goalCallback, this, std::placeholders::_1));
        
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&MultiPathPlanner::startCallback, this, std::placeholders::_1));
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&MultiPathPlanner::mapCallback, this, std::placeholders::_1));
        
        // Three path publishers - one for each strategy
        path_fast_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_fast", 10);
        path_safe_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_safe", 10);
        path_explore_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_explore", 10);
        
        RCLCPP_INFO(this->get_logger(), "Multi-Path Planner ready!");
        RCLCPP_INFO(this->get_logger(), "Will show 3 paths:");
        RCLCPP_INFO(this->get_logger(), "  RED = Fast (direct to goal)");
        RCLCPP_INFO(this->get_logger(), "  GREEN = Safe (avoids obstacles)");
        RCLCPP_INFO(this->get_logger(), "  BLUE = Explore (thorough coverage)");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        collision_checker_->setMap(msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "Map received");
    }
    
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        start_x_ = msg->pose.pose.position.x;
        start_y_ = msg->pose.pose.position.y;
        start_yaw_ = 0;
        has_start_ = true;
        RCLCPP_INFO(this->get_logger(), "Start: (%.2f, %.2f)", start_x_, start_y_);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!collision_checker_->hasMap() || !has_start_) {
            RCLCPP_WARN(this->get_logger(), "Need map and start first!");
            return;
        }
        
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_yaw_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", goal_x_, goal_y_);
        RCLCPP_INFO(this->get_logger(), "Planning 3 paths...");
        
        planAllPaths();
    }
    
    void planAllPaths() {
        pivot_planner::State start(start_x_, start_y_, start_yaw_);
        pivot_planner::State goal(goal_x_, goal_y_, goal_yaw_);
        
        double xmin = -6.5, xmax = 6.5, ymin = -4.0, ymax = 4.0;
        
        // FAST path - aggressive to goal
        RCLCPP_INFO(this->get_logger(), "Planning FAST path...");
        auto path_fast = planWithStrategy(
            start, goal, xmin, xmax, ymin, ymax,
            3.0, 0.3, 0.1,  // theta: high goal, low safety, low exploration
            0.9, 3.0        // alpha, tau: strong bias
        );
        publishPath(path_fast, path_fast_pub_);
        
        //  SAFE path - avoid obstacles
        RCLCPP_INFO(this->get_logger(), "Planning SAFE path...");
        auto path_safe = planWithStrategy(
            start, goal, xmin, xmax, ymin, ymax,
            0.5, 5.0, 0.2,  // theta: low goal, HIGH safety, low exploration
            0.8, 2.0        // alpha, tau
        );
        publishPath(path_safe, path_safe_pub_);
        
        //  EXPLORE path - thorough coverage
        RCLCPP_INFO(this->get_logger(), "Planning EXPLORE path...");
        auto path_explore = planWithStrategy(
            start, goal, xmin, xmax, ymin, ymax,
            0.5, 1.0, 3.0,  // theta: low goal, normal safety, HIGH exploration
            0.7, 1.5        // alpha, tau
        );
        publishPath(path_explore, path_explore_pub_);
        
        RCLCPP_INFO(this->get_logger(), "All paths complete!");
    }
    
    std::vector<pivot_planner::State> planWithStrategy(
        const pivot_planner::State& start,
        const pivot_planner::State& goal,
        double xmin, double xmax, double ymin, double ymax,
        double theta_goal, double theta_safety, double theta_exploration,
        double alpha, double tau)
    {
        pivot_planner::Theta theta(theta_goal, theta_safety, theta_exploration);
        
        auto feat = std::make_unique<pivot_planner::FeatureExtractor>(
            start, goal, xmin, xmax, ymin, ymax);
        feat->setOccupancyGrid(collision_checker_->getMap());
        
        auto base = std::make_unique<pivot_planner::UniformSampler>(
            xmin, xmax, ymin, ymax);
        
        auto pivot = std::make_unique<pivot_planner::PivotSampler>(
            *base, *feat, theta, alpha, tau, 20);
        
        auto planner = std::make_unique<pivot_planner::PivotRRTStar>(
            *pivot, *feat, theta, start, goal, xmin, xmax, ymin, ymax,
            0.2, 0.8, 0.5, collision_checker_);
        
        planner->run(3000);
        
        if (planner->hasReachedGoal()) {
            return planner->getBestPath();
        }
        return std::vector<pivot_planner::State>();
    }
    

    void publishPath(const std::vector<pivot_planner::State>& path,
                 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) {
        auto msg = nav_msgs::msg::Path();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        // Build the ROS path message (same as before)
        for (const auto& s : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = s.x;
            pose.pose.position.y = s.y;
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        // GRAPH + BFS 
        if (!path.empty()) {
            const std::size_t n = path.size();

            // Treat each waypoint as a node in a simple directed graph.
            // Adjacency list: edge from i -> i+1 along the path.
            std::vector<std::vector<int>> adjacency(n);
            for (std::size_t i = 0; i + 1 < n; ++i) {
                adjacency[i].push_back(static_cast<int>(i + 1));
            }

            // BFS using a queue (search algorithm over the graph)
            std::vector<bool> visited(n, false);
            std::queue<int> q;

            visited[0] = true;
            q.push(0);
            int reachable = 0;

            while (!q.empty()) {
                int u = q.front();
                q.pop();
                ++reachable;

                for (int v : adjacency[u]) {
                    if (!visited[v]) {
                        visited[v] = true;
                        q.push(v);
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(),
                        "Graph BFS (assignment demo): reached %d/%zu waypoints",
                        reachable, n);
        }
  
        pub->publish(msg);
    }

    
    std::shared_ptr<pivot_planner::MapCollisionChecker> collision_checker_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_fast_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_safe_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_explore_pub_;
    
    double start_x_, start_y_, start_yaw_;
    double goal_x_, goal_y_, goal_yaw_;
    bool has_start_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiPathPlanner>());
    rclcpp::shutdown();
    return 0;
}

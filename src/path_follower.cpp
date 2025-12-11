#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

class PathFollower : public rclcpp::Node {
public:
    PathFollower() : Node("path_follower"), current_waypoint_idx_(0) {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/pivot_path", 10,
            std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
                
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathFollower::controlLoop, this));
        
        this->declare_parameter("lookahead_distance", 0.5);
        this->declare_parameter("linear_speed", 0.3);
        this->declare_parameter("angular_gain", 2.0);
        this->declare_parameter("waypoint_tolerance", 0.2);
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_gain_ = this->get_parameter("angular_gain").as_double();
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Path Follower ready!");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }
        
        path_ = *msg;
        current_waypoint_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", path_.poses.size());
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                   msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                         msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }
    
    void controlLoop() {
        if (path_.poses.empty()) {
            return;
        }
        
        double dx = path_.poses[current_waypoint_idx_].pose.position.x - current_x_;
        double dy = path_.poses[current_waypoint_idx_].pose.position.y - current_y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < waypoint_tolerance_) {
            current_waypoint_idx_++;
            
            if (current_waypoint_idx_ >= path_.poses.size()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                stopRobot();
                path_.poses.clear();
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu/%zu", 
                       current_waypoint_idx_, path_.poses.size());
        }
        
        size_t lookahead_idx = current_waypoint_idx_;
        for (size_t i = current_waypoint_idx_; i < path_.poses.size(); i++) {
            double ldx = path_.poses[i].pose.position.x - current_x_;
            double ldy = path_.poses[i].pose.position.y - current_y_;
            double ldist = std::sqrt(ldx*ldx + ldy*ldy);
            
            if (ldist >= lookahead_distance_) {
                lookahead_idx = i;
                break;
            }
        }
        
        double target_x = path_.poses[lookahead_idx].pose.position.x;
        double target_y = path_.poses[lookahead_idx].pose.position.y;
        
        double angle_to_target = std::atan2(target_y - current_y_, target_x - current_x_);
        double angle_diff = angle_to_target - current_yaw_;
        
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = linear_speed_;
        cmd.twist.angular.z = angular_gain_ * angle_diff;
        
        cmd_pub_->publish(cmd);
    }
    
    void stopRobot() {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path path_;
    size_t current_waypoint_idx_;
    
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    
    double lookahead_distance_;
    double linear_speed_;
    double angular_gain_;
    double waypoint_tolerance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}
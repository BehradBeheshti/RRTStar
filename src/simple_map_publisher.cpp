#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

class SimpleMapPublisher : public rclcpp::Node {
public:
    SimpleMapPublisher() : Node("simple_map_publisher") {
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimpleMapPublisher::publishMap, this));
        loadMap();
        RCLCPP_INFO(this->get_logger(), "Publishing park map continuously...");
    }

private:
    void loadMap() {
        std::string map_path = "/home/behrad/ros2_ws/src/pivot_planner/maps/park_map.pgm";
        cv::Mat img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map from %s", map_path.c_str());
            return;
        }

        map_.header.frame_id = "map";
        map_.info.resolution = 0.05;
        map_.info.width = img.cols;
        map_.info.height = img.rows;
        map_.info.origin.position.x = -7.0;
        map_.info.origin.position.y = -6.0;
        map_.info.origin.orientation.w = 1.0;

        map_.data.resize(img.cols * img.rows);
        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                int val = img.at<uchar>(img.rows - 1 - i, j);
                if (val > 250) map_.data[i * img.cols + j] = 0;      // free
                else if (val < 5) map_.data[i * img.cols + j] = 100; // occupied
                else map_.data[i * img.cols + j] = -1;                // unknown
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded map: %dx%d @ %.2fm resolution", 
                    map_.info.width, map_.info.height, map_.info.resolution);
    }

    void publishMap() {
        map_.header.stamp = this->now();
        pub_->publish(map_);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
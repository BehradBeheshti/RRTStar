#!/bin/bash
# PIVOT Planner - OpenAI Integration Auto-Installer
# This script adds OpenAI prompt-based planning to your existing PIVOT planner

set -e

echo "================================================"
echo "🤖 PIVOT + OpenAI Integration Installer"
echo "================================================"
echo ""

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -d "src" ]; then
    echo "❌ Error: Please run this script from your pivot_planner directory!"
    echo "Usage:"
    echo "  cd ~/ros2_ws/src/pivot_planner"
    echo "  bash install_openai.sh"
    exit 1
fi

echo "✓ Found pivot_planner package"
echo ""

# Step 1: Install system dependencies
echo "📦 Installing dependencies..."
sudo apt update
sudo apt install -y libcurl4-openssl-dev nlohmann-json3-dev

echo "✓ Dependencies installed"
echo ""

# Step 2: Create the prompt planner source file
echo "📝 Creating prompt_pivot_planner.cpp..."
cat > src/prompt_pivot_planner.cpp << 'EOFSRC'
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include "pivot_planner/pivot_rrt_star.hpp"
#include "pivot_planner/pivot_sampler.hpp"
#include "pivot_planner/feature_extractor.hpp"
#include "pivot_planner/map_collision_checker.hpp"

class PromptPivotPlanner : public rclcpp::Node {
public:
    PromptPivotPlanner() : Node("prompt_pivot_planner") {
        collision_checker_ = std::make_shared<pivot_planner::MapCollisionChecker>();
        
        this->declare_parameter("openai_api_key", "");
        this->declare_parameter("prompt", "balanced navigation");
        this->declare_parameter("model", "gpt-4");
        
        api_key_ = this->get_parameter("openai_api_key").as_string();
        prompt_ = this->get_parameter("prompt").as_string();
        model_ = this->get_parameter("model").as_string();
        
        if (api_key_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "================================================");
            RCLCPP_ERROR(this->get_logger(), "❌ ERROR: No OpenAI API key provided!");
            RCLCPP_ERROR(this->get_logger(), "Usage:");
            RCLCPP_ERROR(this->get_logger(), "  ros2 launch pivot_planner prompt_planner.launch.py \\");
            RCLCPP_ERROR(this->get_logger(), "    openai_api_key:=YOUR_KEY_HERE \\");
            RCLCPP_ERROR(this->get_logger(), "    prompt:=\"be very safe\"");
            RCLCPP_ERROR(this->get_logger(), "================================================");
            rclcpp::shutdown();
            return;
        }
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, 
            std::bind(&PromptPivotPlanner::goalCallback, this, std::placeholders::_1));
        
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&PromptPivotPlanner::startCallback, this, std::placeholders::_1));
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&PromptPivotPlanner::mapCallback, this, std::placeholders::_1));
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pivot_path", 10);
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🤖 Prompt-Based PIVOT Planner Ready!");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Your prompt: \"%s\"", prompt_.c_str());
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Instructions:");
        RCLCPP_INFO(this->get_logger(), "1. Use '2D Pose Estimate' to set START");
        RCLCPP_INFO(this->get_logger(), "2. Use '2D Goal Pose' to set GOAL");
        RCLCPP_INFO(this->get_logger(), "3. Planner will ask OpenAI to interpret your prompt");
        RCLCPP_INFO(this->get_logger(), "4. Watch the path generate!");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        collision_checker_->setMap(msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "✓ Map received: %dx%d", 
                        msg->info.width, msg->info.height);
    }
    
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        start_x_ = msg->pose.pose.position.x;
        start_y_ = msg->pose.pose.position.y;
        start_yaw_ = 2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        has_start_ = true;
        RCLCPP_INFO(this->get_logger(), "✓ Start set: (%.2f, %.2f)", start_x_, start_y_);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!collision_checker_->hasMap()) {
            RCLCPP_WARN(this->get_logger(), "⚠ Waiting for map...");
            return;
        }
        
        if (!has_start_) {
            RCLCPP_WARN(this->get_logger(), "⚠ Please set start pose first!");
            return;
        }
        
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_yaw_ = 2.0 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);
        
        RCLCPP_INFO(this->get_logger(), "✓ Goal set: (%.2f, %.2f)", goal_x_, goal_y_);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🔮 Asking OpenAI to interpret: \"%s\"", prompt_.c_str());
        
        double theta_goal, theta_safety, theta_exploration, alpha, tau;
        if (!computeThetaFromOpenAI(prompt_, theta_goal, theta_safety, theta_exploration, alpha, tau)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to get theta from OpenAI");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🎯 Planning with OpenAI parameters...");
        planPath(theta_goal, theta_safety, theta_exploration, alpha, tau);
    }
    
    bool computeThetaFromOpenAI(const std::string& prompt,
                               double& theta_goal, double& theta_safety, double& theta_exploration,
                               double& alpha, double& tau) {
        std::string system_prompt = buildSystemPrompt(prompt);
        std::string response = callOpenAI(system_prompt);
        
        if (response.empty()) {
            return false;
        }
        
        return parseResponse(response, theta_goal, theta_safety, theta_exploration, alpha, tau);
    }
    
    std::string buildSystemPrompt(const std::string& user_prompt) {
        return R"(You are a robotics motion planning expert. Convert the user's natural language prompt into planning parameters.

Available parameters:
- theta_goal: How much to prioritize reaching the goal directly (range: 0.1 to 5.0)
- theta_safety: How much to avoid obstacles and stay safe (range: 0.1 to 5.0)
- theta_exploration: How much to explore and cover new areas (range: 0.1 to 3.0)
- alpha: Prompt influence strength (range: 0.5 to 0.9, higher = stronger prompt bias)
- tau: Temperature for sampling sharpness (range: 1.0 to 3.0, higher = sharper focus)

User prompt: ")" + user_prompt + R"("

Respond ONLY with valid JSON in this EXACT format (no other text):
{
  "theta_goal": <number>,
  "theta_safety": <number>,
  "theta_exploration": <number>,
  "alpha": <number>,
  "tau": <number>,
  "reasoning": "<brief explanation>"
}

Examples:
"go fast" → {"theta_goal": 4.0, "theta_safety": 0.5, "theta_exploration": 0.2, "alpha": 0.85, "tau": 2.5, "reasoning": "Prioritize speed"}
"be safe" → {"theta_goal": 0.5, "theta_safety": 4.0, "theta_exploration": 0.3, "alpha": 0.8, "tau": 2.0, "reasoning": "Maximize safety"}
"explore thoroughly" → {"theta_goal": 0.3, "theta_safety": 1.0, "theta_exploration": 2.5, "alpha": 0.7, "tau": 1.5, "reasoning": "Thorough exploration"}
"balanced" → {"theta_goal": 1.5, "theta_safety": 1.5, "theta_exploration": 0.8, "alpha": 0.6, "tau": 1.5, "reasoning": "Balanced approach"})";
    }
    
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
        userp->append((char*)contents, size * nmemb);
        return size * nmemb;
    }
    
    std::string callOpenAI(const std::string& prompt) {
        CURL* curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
            return "";
        }
        
        nlohmann::json request_body = {
            {"model", model_},
            {"messages", {
                {{"role", "system"}, {"content", "You are a robotics expert that outputs only JSON."}},
                {{"role", "user"}, {"content", prompt}}
            }},
            {"temperature", 0.3},
            {"max_tokens", 256}
        };
        
        std::string request_str = request_body.dump();
        std::string response_string;
        
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        std::string auth_header = "Authorization: Bearer " + api_key_;
        headers = curl_slist_append(headers, auth_header.c_str());
        
        curl_easy_setopt(curl, CURLOPT_URL, "https://api.openai.com/v1/chat/completions");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_str.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
        
        CURLcode res = curl_easy_perform(curl);
        
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "CURL error: %s", curl_easy_strerror(res));
            return "";
        }
        
        return response_string;
    }
    
    bool parseResponse(const std::string& response_str,
                      double& theta_goal, double& theta_safety, double& theta_exploration,
                      double& alpha, double& tau) {
        try {
            auto response_json = nlohmann::json::parse(response_str);
            std::string content = response_json["choices"][0]["message"]["content"].get<std::string>();
            
            size_t json_start = content.find('{');
            size_t json_end = content.rfind('}');
            if (json_start != std::string::npos && json_end != std::string::npos) {
                content = content.substr(json_start, json_end - json_start + 1);
            }
            
            auto theta_json = nlohmann::json::parse(content);
            
            theta_goal = theta_json["theta_goal"].get<double>();
            theta_safety = theta_json["theta_safety"].get<double>();
            theta_exploration = theta_json["theta_exploration"].get<double>();
            alpha = theta_json["alpha"].get<double>();
            tau = theta_json["tau"].get<double>();
            std::string reasoning = theta_json["reasoning"].get<std::string>();
            
            RCLCPP_INFO(this->get_logger(), "✅ OpenAI Response:");
            RCLCPP_INFO(this->get_logger(), "   theta_goal: %.2f", theta_goal);
            RCLCPP_INFO(this->get_logger(), "   theta_safety: %.2f", theta_safety);
            RCLCPP_INFO(this->get_logger(), "   theta_exploration: %.2f", theta_exploration);
            RCLCPP_INFO(this->get_logger(), "   alpha: %.2f", alpha);
            RCLCPP_INFO(this->get_logger(), "   tau: %.2f", tau);
            RCLCPP_INFO(this->get_logger(), "   💡 %s", reasoning.c_str());
            
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse: %s", e.what());
            return false;
        }
    }
    
    void planPath(double theta_goal, double theta_safety, double theta_exploration,
                 double alpha, double tau) {
        pivot_planner::State start(start_x_, start_y_, start_yaw_);
        pivot_planner::State goal(goal_x_, goal_y_, goal_yaw_);
        
        double xmin = -6.5, xmax = 6.5, ymin = -4.0, ymax = 4.0;
        
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
        
        RCLCPP_INFO(this->get_logger(), "🚀 Planning...");
        planner->run(3000);
        
        if (planner->hasReachedGoal()) {
            RCLCPP_INFO(this->get_logger(), "✅ Path found! Length: %zu nodes", 
                       planner->getBestPath().size());
            publishPath(planner->getBestPath());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠ No path found in 3000 iterations");
        }
    }
    
    void publishPath(const std::vector<pivot_planner::State>& path) {
        auto msg = nav_msgs::msg::Path();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        for (const auto& s : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = s.x;
            pose.pose.position.y = s.y;
            pose.pose.orientation.z = std::sin(s.yaw / 2.0);
            pose.pose.orientation.w = std::cos(s.yaw / 2.0);
            msg.poses.push_back(pose);
        }
        
        path_pub_->publish(msg);
    }
    
    std::shared_ptr<pivot_planner::MapCollisionChecker> collision_checker_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    double start_x_, start_y_, start_yaw_;
    double goal_x_, goal_y_, goal_yaw_;
    bool has_start_ = false;
    
    std::string api_key_;
    std::string prompt_;
    std::string model_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PromptPivotPlanner>());
    rclcpp::shutdown();
    return 0;
}
EOFSRC

echo "✓ Created src/prompt_pivot_planner.cpp"
echo ""

# Step 3: Create launch file
echo "📝 Creating launch/prompt_planner.launch.py..."
cat > launch/prompt_planner.launch.py << 'EOFLAUNCH'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'openai_api_key',
            default_value='',
            description='Your OpenAI API key'
        ),
        
        DeclareLaunchArgument(
            'prompt',
            default_value='balanced navigation',
            description='Natural language prompt for planning behavior'
        ),
        
        DeclareLaunchArgument(
            'model',
            default_value='gpt-4',
            description='OpenAI model to use (gpt-4, gpt-3.5-turbo, etc.)'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        Node(
            package='pivot_planner',
            executable='simple_map_publisher',
            output='screen'
        ),
        
        Node(
            package='pivot_planner',
            executable='prompt_pivot_planner',
            output='screen',
            parameters=[{
                'openai_api_key': LaunchConfiguration('openai_api_key'),
                'prompt': LaunchConfiguration('prompt'),
                'model': LaunchConfiguration('model')
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
EOFLAUNCH

echo "✓ Created launch/prompt_planner.launch.py"
echo ""

# Step 4: Update CMakeLists.txt
echo "📝 Updating CMakeLists.txt..."
if grep -q "prompt_pivot_planner" CMakeLists.txt; then
    echo "⚠ CMakeLists.txt already has prompt_pivot_planner, skipping..."
else
    # Add CURL and JSON to find_package
    sed -i '/find_package(OpenCV REQUIRED)/a find_package(CURL REQUIRED)\nfind_package(nlohmann_json REQUIRED)' CMakeLists.txt
    
    # Add include directories
    sed -i '/include_directories(${OpenCV_INCLUDE_DIRS})/a include_directories(${CURL_INCLUDE_DIRS})' CMakeLists.txt
    
    # Add the new executable before ament_package()
    sed -i '/ament_package()/i \
# Prompt-based Planner with OpenAI\
add_executable(prompt_pivot_planner src/prompt_pivot_planner.cpp)\
target_link_libraries(prompt_pivot_planner \
  pivot_planner_core\
  ${CURL_LIBRARIES}\
  nlohmann_json::nlohmann_json\
)\
ament_target_dependencies(prompt_pivot_planner\
  rclcpp\
  geometry_msgs\
  nav_msgs\
  visualization_msgs\
)\
\
install(TARGETS\
  prompt_pivot_planner\
  DESTINATION lib/${PROJECT_NAME}\
)\
' CMakeLists.txt
    
    echo "✓ Updated CMakeLists.txt"
fi
echo ""

# Step 5: Update package.xml
echo "📝 Updating package.xml..."
if grep -q "libcurl-dev" package.xml; then
    echo "⚠ package.xml already has dependencies, skipping..."
else
    sed -i '/<depend>tf2_geometry_msgs<\/depend>/a \  <depend>libcurl-dev<\/depend>\n  <depend>nlohmann-json-dev<\/depend>' package.xml
    echo "✓ Updated package.xml"
fi
echo ""

# Step 6: Build
echo "🔨 Building package..."
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pivot_planner

echo ""
echo "================================================"
echo "✅ Installation Complete!"
echo "================================================"
echo ""
echo "🚀 To run with OpenAI:"
echo ""
echo "ros2 launch pivot_planner prompt_planner.launch.py \\"
echo "  openai_api_key:=sk-YOUR-KEY-HERE \\"
echo "  prompt:=\"be very safe\""
echo ""
echo "📝 Example prompts to try:"
echo "  - \"be very safe\""
echo "  - \"go as fast as possible\""
echo "  - \"explore thoroughly\""
echo "  - \"balanced navigation\""
echo "  - \"navigate efficiently but prioritize safety\""
echo ""
echo "💡 Then in RViz:"
echo "  1. Click '2D Pose Estimate' to set START"
echo "  2. Click '2D Goal Pose' to set GOAL"
echo "  3. Watch OpenAI interpret your prompt!"
echo "  4. See the path appear!"
echo ""
echo "================================================"

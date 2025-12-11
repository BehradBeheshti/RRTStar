# LLM Integration Guide for PIVOT Planner

This guide shows how to integrate a Large Language Model to map natural language prompts to theta (θ) weight vectors.

## Overview

Current: You manually set theta weights in YAML
```yaml
theta_goal: 1.0
theta_safety: 1.0
theta_exploration: 0.5
```

Goal: User says "Navigate carefully avoiding obstacles" → System computes theta automatically

## Architecture Options

### Option 1: Pre-computed Prompt Templates (Simplest)

Create a lookup table of common prompts:

```cpp
// In pivot_planner_node.cpp
std::map<std::string, Theta> prompt_templates_ = {
    {"fast", Theta(2.0, 0.5, 0.1)},
    {"quick", Theta(2.0, 0.5, 0.1)},
    {"safe", Theta(0.5, 2.0, 0.3)},
    {"careful", Theta(0.5, 2.0, 0.3)},
    {"explore", Theta(0.3, 1.0, 2.0)},
    {"thorough", Theta(0.3, 1.0, 2.0)},
    {"balanced", Theta(1.0, 1.0, 1.0)},
};

Theta parsePrompt(const std::string& prompt) {
    for (const auto& [keyword, theta] : prompt_templates_) {
        if (prompt.find(keyword) != std::string::npos) {
            return theta;
        }
    }
    return Theta(1.0, 1.0, 1.0); // default
}
```

**Pros**: Simple, no external dependencies, fast
**Cons**: Limited flexibility, requires manual tuning

### Option 2: LLM API Service (Recommended)

Query an LLM (Claude, GPT, etc.) to map prompt → theta

#### Step 1: Add Service Definition

Create `srv/ComputeTheta.srv`:
```
string natural_language_prompt
---
float64 theta_goal
float64 theta_safety
float64 theta_exploration
bool success
string message
```

#### Step 2: Create LLM Client Node

```cpp
// llm_theta_service.cpp
#include <rclcpp/rclcpp.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include "pivot_planner/srv/compute_theta.hpp"

class LLMThetaService : public rclcpp::Node {
public:
    LLMThetaService() : Node("llm_theta_service") {
        service_ = this->create_service<pivot_planner::srv::ComputeTheta>(
            "compute_theta",
            std::bind(&LLMThetaService::handleRequest, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        this->declare_parameter("api_key", "");
        this->declare_parameter("model", "claude-3-5-sonnet-20241022");
    }

private:
    void handleRequest(
        const std::shared_ptr<pivot_planner::srv::ComputeTheta::Request> request,
        std::shared_ptr<pivot_planner::srv::ComputeTheta::Response> response)
    {
        std::string prompt = buildSystemPrompt(request->natural_language_prompt);
        std::string llm_response = queryLLM(prompt);
        
        // Parse JSON response from LLM
        auto theta = parseTheta(llm_response);
        
        response->theta_goal = theta.w_goal;
        response->theta_safety = theta.w_safety;
        response->theta_exploration = theta.w_exploration;
        response->success = true;
        response->message = "Successfully computed theta from prompt";
    }
    
    std::string buildSystemPrompt(const std::string& user_prompt) {
        return R"(
You are a motion planning expert. Convert the user's natural language 
description into feature weights for a robot planner.

Available features:
- goal_progress: How close to reaching the goal (0-2 scale)
- safety: How far from obstacles (0-2 scale)
- exploration: How much to explore new areas (0-2 scale)

User prompt: )" + user_prompt + R"(

Respond ONLY with valid JSON in this exact format:
{
  "theta_goal": <number>,
  "theta_safety": <number>,
  "theta_exploration": <number>,
  "reasoning": "<brief explanation>"
}

Examples:
"Navigate quickly" -> {"theta_goal": 2.0, "theta_safety": 0.5, "theta_exploration": 0.1}
"Be very careful" -> {"theta_goal": 0.5, "theta_safety": 2.0, "theta_exploration": 0.3}
"Explore thoroughly" -> {"theta_goal": 0.3, "theta_safety": 1.0, "theta_exploration": 2.0}
)";
    }
    
    std::string queryLLM(const std::string& prompt) {
        // Implement API call to Claude/GPT/etc.
        // See example below
        return callAnthropicAPI(prompt);
    }
    
    std::string callAnthropicAPI(const std::string& prompt) {
        std::string api_key = this->get_parameter("api_key").as_string();
        
        nlohmann::json request_body = {
            {"model", this->get_parameter("model").as_string()},
            {"max_tokens", 256},
            {"messages", {{
                {"role", "user"},
                {"content", prompt}
            }}}
        };
        
        // Use libcurl or similar to make HTTP request
        // ... (implementation details)
        
        return response_text;
    }
    
    Theta parseTheta(const std::string& json_str) {
        auto j = nlohmann::json::parse(json_str);
        return Theta(
            j["theta_goal"].get<double>(),
            j["theta_safety"].get<double>(),
            j["theta_exploration"].get<double>()
        );
    }
    
    rclcpp::Service<pivot_planner::srv::ComputeTheta>::SharedPtr service_;
};
```

#### Step 3: Modify Planner Node to Use Service

```cpp
// In PivotPlannerNode class
class PivotPlannerNode : public rclcpp::Node {
public:
    PivotPlannerNode() : Node("pivot_planner_node") {
        // ... existing initialization ...
        
        // Add service client
        theta_client_ = this->create_client<pivot_planner::srv::ComputeTheta>(
            "compute_theta");
        
        // Add prompt parameter
        this->declare_parameter("natural_language_prompt", "");
        
        // If prompt provided, use LLM to compute theta
        std::string prompt = this->get_parameter("natural_language_prompt").as_string();
        if (!prompt.empty()) {
            computeThetaFromPrompt(prompt);
        }
    }

private:
    void computeThetaFromPrompt(const std::string& prompt) {
        // Wait for service
        while (!theta_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for theta computation service...");
        }
        
        // Create request
        auto request = std::make_shared<pivot_planner::srv::ComputeTheta::Request>();
        request->natural_language_prompt = prompt;
        
        // Call service
        auto future = theta_client_->async_send_request(request);
        
        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), 
                    "Computed theta from prompt: [%.2f, %.2f, %.2f]",
                    response->theta_goal, 
                    response->theta_safety,
                    response->theta_exploration);
                
                // Update planner with new theta
                pivot_planner::Theta new_theta(
                    response->theta_goal,
                    response->theta_safety,
                    response->theta_exploration
                );
                
                pivot_sampler_->setTheta(new_theta);
            }
        }
    }
    
    rclcpp::Client<pivot_planner::srv::ComputeTheta>::SharedPtr theta_client_;
};
```

#### Step 4: Launch with Natural Language

```bash
ros2 launch pivot_planner pivot_planner.launch.py \
    natural_language_prompt:="Navigate quickly but be careful near obstacles"
```

### Option 3: Embedded LLM (Advanced)

Use llama.cpp or similar to run LLM locally:

```cpp
#include <llama.h>

class EmbeddedLLMTheta {
public:
    EmbeddedLLMTheta(const std::string& model_path) {
        // Load model
        params_ = llama_context_default_params();
        model_ = llama_load_model_from_file(model_path.c_str(), params_);
    }
    
    Theta computeTheta(const std::string& prompt) {
        std::string full_prompt = buildPrompt(prompt);
        std::string response = generate(full_prompt);
        return parseTheta(response);
    }
    
private:
    llama_context* model_;
    llama_context_params params_;
};
```

**Pros**: No API costs, works offline, low latency
**Cons**: Requires local GPU/CPU, more complex setup

## Example Prompt → Theta Mappings

Based on the LLM's understanding:

| Natural Language Prompt | θ_goal | θ_safety | θ_exploration |
|------------------------|--------|----------|---------------|
| "Navigate quickly to the goal" | 2.0 | 0.5 | 0.1 |
| "Get there fast" | 2.0 | 0.3 | 0.1 |
| "Be very careful and safe" | 0.5 | 2.0 | 0.3 |
| "Avoid obstacles at all costs" | 0.3 | 2.0 | 0.2 |
| "Explore the environment thoroughly" | 0.3 | 1.0 | 2.0 |
| "Map the entire space" | 0.2 | 1.0 | 2.0 |
| "Balanced navigation" | 1.0 | 1.0 | 1.0 |
| "Fast but not reckless" | 1.5 | 1.2 | 0.3 |
| "Cautious exploration" | 0.5 | 1.5 | 1.5 |

## Dynamic Theta Updates

You can update theta during planning:

```cpp
// Subscribe to prompt updates
prompt_sub_ = this->create_subscription<std_msgs::msg::String>(
    "new_prompt", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        auto new_theta = computeThetaFromPrompt(msg->data);
        pivot_sampler_->setTheta(new_theta);
        RCLCPP_INFO(this->get_logger(), "Updated theta from new prompt");
    });
```

```bash
# Send new prompt while planning
ros2 topic pub /new_prompt std_msgs/msg/String \
    "data: 'Switch to exploration mode'"
```

## Complete Example System

```
┌─────────────────┐
│  User Input     │
│  "Be careful"   │
└────────┬────────┘
         │
         v
┌─────────────────┐
│  LLM Service    │──> Query Claude/GPT API
│  (ROS2 Node)    │
└────────┬────────┘
         │ θ = [0.5, 2.0, 0.3]
         v
┌─────────────────┐
│  PIVOT Planner  │──> Plans with biased sampling
│  (ROS2 Node)    │
└────────┬────────┘
         │ Path
         v
┌─────────────────┐
│  Robot/Sim      │
└─────────────────┘
```

## Testing LLM Integration

1. **Unit test prompt templates**:
```bash
ros2 service call /compute_theta pivot_planner/srv/ComputeTheta \
    "{natural_language_prompt: 'Navigate carefully'}"
```

2. **Compare LLM vs manual theta**:
- Run with LLM-generated theta
- Run with manually tuned theta
- Compare behaviors

3. **Robustness testing**:
- Ambiguous prompts: "Go over there"
- Conflicting goals: "Fast and extremely safe"
- Invalid prompts: "Purple elephant"

## Adding More Features

To support richer prompts, extend features:

```cpp
struct Features {
    double goal_progress;
    double safety;
    double exploration;
    double speed;           // NEW: prefer faster paths
    double smoothness;      // NEW: prefer smooth curves
    double energy;          // NEW: energy efficiency
};

struct Theta {
    double w_goal;
    double w_safety;
    double w_exploration;
    double w_speed;
    double w_smoothness;
    double w_energy;
};
```

Then prompts like "Navigate smoothly and efficiently" map to higher smoothness and energy weights.

## Next Steps

1. Implement Option 1 (templates) first - simplest
2. Add Option 2 (LLM service) when ready for full NL
3. Benchmark different prompts
4. Tune LLM system prompt for best theta generation
5. Consider fine-tuning small model on prompt→theta pairs

## Resources

- **Anthropic API**: https://docs.anthropic.com/
- **OpenAI API**: https://platform.openai.com/docs
- **llama.cpp**: https://github.com/ggerganov/llama.cpp
- **ROS2 Services**: https://docs.ros.org/en/jazzy/Tutorials/Services.html

Your PIVOT implementation is ready for this integration! 🚀

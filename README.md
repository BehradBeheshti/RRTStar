# RRT* with Prompt: Natural Language Motion Planning

A natural language-driven motion planning system for mobile robots. Tell the robot how you want it to behave, and it adapts its path planning strategy accordingly.

## What It Does

RRT* with Prompt uses OpenAI's API to interpret natural language prompts and converts them into path planning parameters. Different prompts produce dramatically different behaviors:

- **"be fast"** → Direct, aggressive paths that prioritize speed
- **"be safe"** → Cautious paths that maintain distance from obstacles
- **"explore"** → Curious paths that cover new areas

## Demo Videos

- [Fast mode demonstration](https://youtu.be/gSnNUvaxkLY) - prompt: "be fast"
- [Safe mode demonstration](https://youtu.be/IlJILYb7LDg) - prompt: "be safe"

## How It Works

1. You give the robot a natural language prompt
2. OpenAI interprets it into planning parameters (theta_goal, theta_safety, theta_exploration)
3. A custom RRT* planner uses exponentially-tilted sampling to bias exploration
4. The robot follows the generated path in Gazebo simulation

## Tech Stack

- **ROS2 Jazzy** for robot middleware
- **Gazebo** for simulation
- **OpenAI API** for prompt interpretation
- **Custom RRT*** implementation with feature-based sampling
- **Clearpath Jackal** robot platform

## Key Features

- Deterministic path planning (same prompt = same path)
- Adaptive iteration counts based on prompt type
- Path smoothing to handle sharp turns
- Real-time visualization in RViz

## Quick Start
```bash
ros2 launch pivot_planner gazebo_demo.launch.py \
  use_prompt:=true \
  openai_api_key:=YOUR_OPENAI_KEY_HERE \
  prompt:="go as safe as you can"
```

Then use RViz to set start (2D Pose Estimate) and goal (2D Goal Pose).

Try different prompts like:
- `"be extremely fast"`
- `"explore the environment"`
- `"balance speed and safety"`

---
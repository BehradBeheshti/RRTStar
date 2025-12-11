# PIVOT Planner - Quick Start Guide

## TL;DR - Get Running in 3 Steps

### 1. Install
```bash
cd pivot_planner_ros2
./install.sh
```

Or manually:
```bash
mkdir -p ~/ros2_ws/src
cp -r pivot_planner_ros2 ~/ros2_ws/src/pivot_planner
cd ~/ros2_ws
colcon build --packages-select pivot_planner
```

### 2. Source
```bash
source ~/ros2_ws/install/setup.bash
```

### 3. Run
```bash
ros2 launch pivot_planner pivot_planner.launch.py
```

You should see:
- RViz window showing the planning space
- Blue tree growing from (0,0) toward (8,8)
- Orange path appearing once goal is reached

## Understanding the Output

Terminal output:
```
[INFO] PIVOT Planner Node initialized
[INFO] Start: (0.00, 0.00, 0.00)
[INFO] Goal: (8.00, 8.00, 0.00)
[INFO] Alpha: 0.50, Tau: 1.00
[INFO] Theta: [1.00, 1.00, 0.50]
[INFO] Iteration 100, Tree size: 98
[INFO] Iteration 200, Tree size: 195
...
[INFO] Planning complete. Iterations: 2000, Tree size: 1876
[INFO] Goal reached!
```

RViz visualization:
- **Blue lines**: RRT* tree (all explored paths)
- **Orange line**: Best path found

## Experiment with Different "Prompts"

Edit `config/pivot_params.yaml` and change theta values:

### Fast to Goal
```yaml
theta_goal: 2.0        # High priority on reaching goal
theta_safety: 0.5      # Lower priority on safety
theta_exploration: 0.1 # Minimal exploration
tau: 2.0               # Strong bias
alpha: 0.7             # More prompt influence
```

### Safe Navigation  
```yaml
theta_goal: 0.5        # Lower priority on speed
theta_safety: 2.0      # High priority on safety
theta_exploration: 0.3
tau: 1.5
alpha: 0.6
```

### Thorough Exploration
```yaml
theta_goal: 0.3
theta_safety: 1.0
theta_exploration: 2.0 # High priority on exploration
tau: 1.0
alpha: 0.5
```

After editing, relaunch:
```bash
ros2 launch pivot_planner pivot_planner.launch.py
```

## Comparing to Vanilla RRT*

To see vanilla RRT* behavior (no prompt influence):
```yaml
alpha: 0.0  # Pure base sampler
```

To see maximum prompt influence:
```yaml
alpha: 0.9  # Heavy prompt bias (still maintains completeness)
tau: 3.0    # Very strong bias
```

## Understanding Parameters

### α (alpha) - Mixture Weight
- Controls balance between exploration and prompt-directed behavior
- `α = 0`: Pure RRT* (ignores prompt)
- `α = 0.5`: Balanced (50/50 mix)
- `α = 0.9`: Strong prompt bias

### τ (tau) - Temperature
- Controls sharpness of utility-based sampling
- `τ = 0.1`: Soft bias (almost uniform within prompt-biased portion)
- `τ = 1.0`: Moderate bias
- `τ = 5.0`: Sharp bias (samples almost exclusively high-utility regions)

### θ (theta) - Feature Weights
- Encodes task preferences
- Each weight says "how much do I care about this feature"
- Normalized features are in [0,1], so weights are comparable

## Troubleshooting

### "ROS2 not found"
Make sure ROS2 Jazzy is installed and sourced:
```bash
source /opt/ros/jazzy/setup.bash
```

### "Package not found"
Make sure you've built and sourced:
```bash
cd ~/ros2_ws
colcon build --packages-select pivot_planner
source install/setup.bash
```

### "No path found"
Try:
1. Increase `max_iterations` (default: 2000)
2. Decrease `alpha` (more exploration)
3. Check start/goal are within bounds

### RViz doesn't show anything
1. Check that Fixed Frame is set to "map"
2. Add Path display with topic `/pivot_path`
3. Add MarkerArray display with topic `/pivot_tree`

## Next Steps

1. **Add obstacles**: Modify `FeatureExtractor::getObstacleDistance()` to use real obstacle map
2. **Try different start/goals**: Edit YAML file
3. **Enable speculation**: Set `speculation_enabled: true` to see look-ahead rejection
4. **Integrate with nav2**: Create planner plugin wrapper

## File Structure Quick Reference

```
pivot_planner_ros2/
├── install.sh                  # Run this first
├── README.md                   # Full documentation
├── QUICKSTART.md              # This file
├── package.xml                 # ROS2 package manifest
├── CMakeLists.txt             # Build configuration
├── include/pivot_planner/      # Header files
│   ├── types.hpp              # Data structures
│   ├── feature_extractor.hpp  # φ(x) computation
│   ├── pivot_sampler.hpp      # PIVOT mixture sampler
│   └── pivot_rrt_star.hpp     # Main planner
├── src/                       # Implementation files
│   ├── *.cpp
│   └── pivot_planner_node.cpp # ROS2 node
├── launch/
│   └── pivot_planner.launch.py # Launch file
└── config/
    ├── pivot_params.yaml      # Edit this for experiments!
    └── pivot.rviz             # RViz config
```

## Getting Help

Check the full README.md for:
- Architecture details
- Theory and math
- Extension guides
- API reference

Happy planning! 🚀

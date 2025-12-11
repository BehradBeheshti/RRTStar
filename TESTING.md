# PIVOT Planner - Testing Checklist

Use this checklist to verify the implementation works correctly.

## ✅ Build and Installation

- [ ] Package builds without errors: `colcon build --packages-select pivot_planner`
- [ ] No compiler warnings (or only minor ones)
- [ ] Installation script works: `./install.sh`
- [ ] Package can be sourced: `source install/setup.bash`
- [ ] Node can be found: `ros2 pkg list | grep pivot`

## ✅ Basic Functionality

- [ ] Node launches: `ros2 launch pivot_planner pivot_planner.launch.py`
- [ ] No immediate crashes or errors
- [ ] Terminal shows initialization message
- [ ] RViz window opens
- [ ] Planning iterations start counting up

## ✅ Visualization

- [ ] RViz shows "map" frame
- [ ] Blue tree appears and grows
- [ ] Tree originates from start point (0, 0)
- [ ] Tree grows toward goal (8, 8)
- [ ] Orange path appears when goal is reached
- [ ] Tree updates in real-time

## ✅ Planning Behavior

- [ ] Tree grows from start
- [ ] Eventually reaches goal (within max_iterations)
- [ ] Terminal shows "Goal reached!" message
- [ ] Path is visible from start to goal
- [ ] No crashes during planning

## ✅ Vanilla RRT* Mode (alpha = 0)

Edit config: `alpha: 0.0`, then launch

- [ ] Tree grows uniformly (no bias)
- [ ] Eventually finds path
- [ ] Behavior looks like standard RRT*

## ✅ PIVOT Mode (alpha > 0)

Edit config: `alpha: 0.5`, then launch

- [ ] Tree shows some directional bias
- [ ] Different from vanilla RRT* behavior
- [ ] Still reaches goal

## ✅ Different Prompt Configurations

### "Fast to Goal"
Config: `theta_goal: 2.0`, `theta_safety: 0.5`, `alpha: 0.7`, `tau: 2.0`

- [ ] Tree grows more directly toward goal
- [ ] Less exploration in other directions
- [ ] Finds path quickly

### "Safe Navigation"
Config: `theta_safety: 2.0`, `theta_goal: 0.5`, `alpha: 0.6`, `tau: 1.5`

- [ ] Tree avoids edges of planning space
- [ ] More conservative exploration
- [ ] Still finds path

### "Thorough Exploration"
Config: `theta_exploration: 2.0`, `theta_goal: 0.3`, `alpha: 0.5`, `tau: 1.0`

- [ ] Tree spreads out more
- [ ] Visits more diverse regions
- [ ] Takes longer to reach goal

## ✅ Parameter Variations

### Alpha Sweep (tau=1.0)
- [ ] α=0.0: Uniform exploration
- [ ] α=0.5: Balanced
- [ ] α=0.9: Strong prompt bias

### Tau Sweep (alpha=0.5)
- [ ] τ=0.1: Soft bias (almost uniform)
- [ ] τ=1.0: Moderate bias
- [ ] τ=3.0: Sharp bias (very directed)

## ✅ Edge Cases

- [ ] Start = Goal: Handles gracefully
- [ ] Large bounds (xmax=100): Still works
- [ ] Small step_size (0.1): Finer tree
- [ ] Large step_size (2.0): Coarser tree
- [ ] High iterations (10000): Doesn't crash
- [ ] Low iterations (100): May not find goal but doesn't crash

## ✅ ROS2 Integration

- [ ] Topics exist: `ros2 topic list`
  - [ ] `/pivot_path`
  - [ ] `/pivot_tree`
- [ ] Path published: `ros2 topic echo /pivot_path`
- [ ] Tree markers published: `ros2 topic echo /pivot_tree`
- [ ] Parameters readable: `ros2 param list`
- [ ] Can get parameters: `ros2 param get /pivot_planner_node alpha`

## ✅ Speculation (Optional)

Edit config: `speculation_enabled: true`, `speculation_k: 3`

- [ ] Planning still works
- [ ] May converge faster (fewer bad branches)
- [ ] Terminal may show fewer tree nodes for same iterations

## ✅ Performance

- [ ] Maintains ~10Hz update rate
- [ ] No significant lag in visualization
- [ ] CPU usage reasonable (<100% on one core)
- [ ] Memory usage stable (no leaks over time)

## ✅ Code Quality

- [ ] Headers compile independently
- [ ] No segfaults during normal operation
- [ ] Clean shutdown (Ctrl+C works)
- [ ] No memory leaks (check with valgrind if available)

## 🐛 Common Issues and Fixes

### Issue: "Package not found"
**Fix**: Make sure you've sourced the workspace
```bash
source ~/ros2_ws/install/setup.bash
```

### Issue: RViz doesn't show anything
**Fix**: 
1. Check Fixed Frame is "map"
2. Manually add Path and MarkerArray displays
3. Set correct topics

### Issue: No path found within max_iterations
**Fix**: 
1. Increase max_iterations (e.g., 5000)
2. Decrease alpha (more exploration)
3. Increase neighbor_radius

### Issue: Build fails
**Fix**:
1. Make sure ROS2 Jazzy is sourced
2. Check all dependencies installed
3. Clean build: `rm -rf build install log && colcon build`

## 📊 Expected Results

After running with default parameters (2000 iterations):

- **Tree size**: ~1800-1900 nodes
- **Goal reached**: Yes (within 0.3 tolerance)
- **Time to goal**: Varies (500-1500 iterations typically)
- **Path quality**: Improved over iterations (RRT* rewiring)

## ✅ Ready for Research

If all above items checked:

- [ ] Implementation is correct
- [ ] Ready for experiments
- [ ] Can generate comparison data
- [ ] Can test different prompts
- [ ] Can integrate with real robot

## 📝 Testing Notes

Date: ___________

Tester: ___________

Platform: ___________

Results:
- [ ] All tests passed
- [ ] Some tests failed (document below)
- [ ] Needs debugging

Issues found:
```
[Write any issues or unexpected behaviors here]
```

Performance observations:
```
[Note any performance characteristics]
```

Suggestions for improvement:
```
[Ideas for enhancements]
```

# PIVOT Planner - Quick Reference Card

## 🚀 Installation (30 seconds)
```bash
cd pivot_planner_ros2 && ./install.sh
source ~/ros2_ws/install/setup.bash
```

## ▶️ Run
```bash
ros2 launch pivot_planner pivot_planner.launch.py
```

## 🎛️ Key Parameters (edit config/pivot_params.yaml)

### PIVOT Controls
| Parameter | Range | Effect |
|-----------|-------|--------|
| `alpha` | 0.0-1.0 | 0=pure RRT*, 1=pure prompt-biased |
| `tau` | 0.1-5.0 | Higher = sharper utility bias |
| `batch_size` | 10-50 | More candidates = better approximation |

### Task Specification (Theta)
| Weight | Meaning | High Value → |
|--------|---------|--------------|
| `theta_goal` | Goal importance | Direct paths |
| `theta_safety` | Obstacle avoidance | Conservative |
| `theta_exploration` | Visit new areas | Thorough coverage |

## 📋 Preset Configurations

**Fast Navigation:**
```yaml
alpha: 0.7, tau: 2.0
theta_goal: 2.0, theta_safety: 0.5, theta_exploration: 0.1
```

**Safe Navigation:**
```yaml
alpha: 0.6, tau: 1.5
theta_goal: 0.5, theta_safety: 2.0, theta_exploration: 0.3
```

**Thorough Exploration:**
```yaml
alpha: 0.5, tau: 1.0
theta_goal: 0.3, theta_safety: 1.0, theta_exploration: 2.0
```

**Vanilla RRT*** (no PIVOT):
```yaml
alpha: 0.0
```

## 📊 Topics
| Topic | Type | Content |
|-------|------|---------|
| `/pivot_path` | Path | Best path found |
| `/pivot_tree` | MarkerArray | RRT* tree |

## 🔍 Debugging

**No path found?**
→ Increase `max_iterations` or decrease `alpha`

**Tree looks wrong in RViz?**
→ Check Fixed Frame = "map"

**Build fails?**
→ `source /opt/ros/jazzy/setup.bash` first

## 📐 Math Quick Reference

```
φ(x) ∈ [0,1]^m              Features (normalized)
U_θ(x) = θ^T φ(x)            Utility function
p_τ(x) ∝ exp(τU_θ)p_0        Tilted distribution
p_{α,τ} = (1-α)p_0 + αp_τ    PIVOT mixture
```

## 📚 Documentation Files

- `QUICKSTART.md` → 3-step getting started
- `README.md` → Full documentation
- `TESTING.md` → Verification checklist
- `LLM_INTEGRATION.md` → Natural language guide

## 🎯 Common Use Cases

**Compare to baseline:**
1. Run with `alpha: 0.0` (RRT*)
2. Run with `alpha: 0.5` (PIVOT)
3. Compare tree structures

**Test prompt:**
1. Choose theta weights for task
2. Set appropriate alpha, tau
3. Observe biased behavior

**Add obstacles:**
→ See README.md "Extending" section

## ⚡ Quick Tips

- Start with `alpha: 0.0` to verify base planner works
- Use `alpha: 0.3-0.7` for balanced PIVOT behavior
- Higher `tau` → stronger bias (but slower sampling)
- `batch_size: 20` is good default
- Check RViz to see what's happening
- Edit YAML, no recompilation needed!

## 🆘 Help

Full docs: `cat README.md`
Test: `cat TESTING.md`
Issues: Check TESTING.md troubleshooting section

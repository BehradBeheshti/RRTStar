#!/usr/bin/env python3
import numpy as np
import cv2

# Map parameters matching park_demo.sdf
width = 280   # 14m / 0.05 resolution
height = 240  # 12m / 0.05 resolution
resolution = 0.05

# Create white (free) map
map_img = np.ones((height, width), dtype=np.uint8) * 255

def world_to_grid(x, y):
    """Convert world coordinates to grid coordinates"""
    grid_x = int((x - (-7.0)) / resolution)
    grid_y = int((y - (-6.0)) / resolution)
    return grid_x, grid_y

def draw_box(img, x, y, w, h, color=0):
    """Draw a box obstacle"""
    gx, gy = world_to_grid(x, y)
    gw = int(w / resolution)
    gh = int(h / resolution)
    cv2.rectangle(img, 
                  (gx - gw//2, gy - gh//2), 
                  (gx + gw//2, gy + gh//2), 
                  color, -1)

def draw_circle(img, x, y, radius, color=0):
    """Draw a circular obstacle"""
    gx, gy = world_to_grid(x, y)
    gr = int(radius / resolution)
    cv2.circle(img, (gx, gy), gr, color, -1)

# Draw obstacles from park_demo.sdf

# Bench (left side) - at (-3.5, 0)
draw_box(map_img, -2.5, 0.5, 1.5, 0.4, 0)

# Tree (center-left) - at (-2, -1), trunk radius 0.25 ONLY (not leaves)
draw_circle(map_img, -2, -1, 0.3, 0)

# Lamp post 1 - at (3.5, 3), radius 0.08
draw_circle(map_img, 3.5, 3, 0.15, 0)

# Lamp post 2 - at (3.5, -3), radius 0.08
draw_circle(map_img, 3.5, -3, 0.15, 0)

# Trash bin - at (-4, -3), radius 0.3
draw_circle(map_img, -4, -3, 0.35, 0)

# Flower bed - at (4, 0), 1.5x1.5
draw_box(map_img, 4, 0, 1.5, 1.5, 0)

# Picnic table - at (0, -5), 2x1
draw_box(map_img, 0, -5, 2, 1.2, 0)

# Boundary walls
draw_box(map_img, 0, 6, 14, 0.3, 0)      # top
draw_box(map_img, 0, -6, 14, 0.3, 0)     # bottom
draw_box(map_img, -7, 0, 0.3, 12, 0)     # left
draw_box(map_img, 7, 0, 0.3, 12, 0)      # right

# Flip vertically (pgm convention)
map_img = cv2.flip(map_img, 0)

# Save
output_path = "/home/behrad/ros2_ws/src/pivot_planner/maps/park_map.pgm"
cv2.imwrite(output_path, map_img)
print(f"Saved park map: {output_path}")
print(f"Size: {width}x{height}")
print(f"Resolution: {resolution}m/pixel")
print(f"Origin: [-7.0, -6.0, 0.0]")

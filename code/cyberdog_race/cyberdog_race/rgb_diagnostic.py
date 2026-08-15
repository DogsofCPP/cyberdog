#!/usr/bin/env python3
"""
Quick diagnostic tool to see what colors are in the simulation camera feed.
Run this while the race simulation is running to tune HSV thresholds.

Usage:
    # Terminal 1: run the race sim
    # Terminal 2:
    source /opt/ros/galactic/setup.bash
    python3 -c "
import rclpy
rclpy.init()
from cyberdog_race.perception.ros2_perception import ROS2Perception
import cv2, numpy as np

node = ROS2Perception()
rclpy.spin_once(node, timeout_sec=2.0)

rgb = node.latest_rgb
if rgb is None:
    print('No RGB yet')
else:
    print(f'Image shape: {rgb.shape}')

    # Compute mean HSV of the whole image
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:,:,0].astype(float), hsv[:,:,1].astype(float), hsv[:,:,2].astype(float)
    print(f'HSV stats: H={h.mean():.1f}±{h.std():.1f}  S={s.mean():.1f}±{s.std():.1f}  V={v.mean():.1f}±{v.std():.1f}')
    print(f'H range: [{h.min():.0f}, {h.max():.0f}]  S range: [{s.min():.0f}, {s.max():.0f}]  V range: [{v.min():.0f}, {v.max():.0f}]')

    # Test current thresholds
    yellow = cv2.inRange(hsv, np.array([15,80,80]), np.array([40,255,255]))
    orange = cv2.inRange(hsv, np.array([5,140,140]), np.array([30,255,255]))
    blue   = cv2.inRange(hsv, np.array([75,40,60]),  np.array([140,255,255]))
    print(f'Pixels in current thresholds -> yellow:{yellow.sum()//255}  orange:{orange.sum()//255}  blue:{blue.sum()//255}')

    # Scan for dominant colors: show which hue bins have most pixels
    print('Dominant hue bins (H, count):')
    hist = cv2.calcHist([hsv], [0], None, [18], [0, 180])
    for i, v in enumerate(hist.flatten()):
        if v > rgb.shape[0]*rgb.shape[1]*0.01:  # >1% of pixels
            print(f'  H={i*10}-{(i+1)*10}: {int(v)} px')
    cv2.imwrite('/home/rgb_diagnostic.png', rgb)
    print('Saved rgb_diagnostic.png')
"

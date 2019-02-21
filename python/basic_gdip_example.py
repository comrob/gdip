#!/usr/bin/env python3

# Test for Wrapper for GDIP library libGDIP.so

##################################################
# Imports
##################################################

import os, sys, math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch

# import the Python wrapper
from dubinswrapper import DubinsWrapper as dubins

##################################################
# Settings
##################################################

# Turning radius
turning_radius = 1.0
# Step of the theta2 angle in the visualization
step = 2 * math.pi / 100
# Step for visualization
step_size = 0.01 * turning_radius
# Wait time for the animation
wait_time = 0.01
# Save figures into a directory "images"
save_figures = False
if len(sys.argv) > 1:
    if "-save" in sys.argv[1]:
        save_figures = True
if save_figures:
    os.makedirs("images", exist_ok=True)
# Sensing radius in case of GDIP solution
sensing_radius_gdip = 0.5

# End points
point1 = [0.0, 0.0]
point2 = [5.0, 0.0]

# First angle interval (the second is changed)
interval1 = [1.0, 0.5]

# Enable only specific parts
DUB = False
DIP = False
GDIP = True 

#matplotlib.rcParams['text.usetex'] = True

##################################################
# Functions
##################################################

def plot_points(points, specs = "b", zorder = 1):
    x_val = [x[0] for x in points]
    y_val = [x[1] for x in points]
    return plt.plot(x_val, y_val, specs, zorder = zorder)

def plot_interval(point, interval, sensing_radius):
    ax = plt.gca()
    circle = Circle(point, sensing_radius, facecolor="yellow", edgecolor="orange", linewidth=1, alpha=0.2)
    ax.add_patch(circle)

    for angle in [interval[0], interval[0] + interval[1]]:
        last = point + turning_radius * np.array([math.cos(angle), math.sin(angle)])
        points = [point, last]
        plot_points(points, specs = "y-", zorder = 1)

def plot_arrow(px, py, angle, alpha = 1):
    rad = 0.7 * turning_radius
    dx = rad * math.cos(angle)
    dy = rad * math.sin(angle)
    plt.arrow(px, py, dx, dy, head_width=0.05, head_length=0.1, fc="k", ec="k", zorder=2, alpha=alpha)
    plt.plot([px, px + dx], [py, py + dy], "k", visible=False)

def plot_figure(samples, point1, point2, title, sensing_radius = 0, interval1 = None, interval2 = None):
    # Clear and basic settings
    plt.clf()
    plt.axis("equal")
    plt.title(title)
    plt.xlabel("x [-]")
    plt.ylabel("y [-]")
    plt.ylim([-3, 3])
    plt.xlim([-1.5, 6.5])
    plt.grid(alpha=0.2)

    # Plot regions based on sensing radius
    if interval1 is not None:
        plot_interval(point1, interval1, sensing_radius)
    if interval2 is not None:
        plot_interval(point2, interval2, sensing_radius)

    # Plot samples of the Dubins maneuver
    ps = plot_points(samples)
    plt.legend([ps[0]], ["Dubins maneuver"])

    # First arrow
    p1 = samples[0]
    p2 = samples[1]
    angle = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
    plot_arrow(p1[0], p1[1], angle)
    plot_arrow(point1[0], point1[1], angle, 0.3)
    plt.plot(p1[0], p1[1], "k.")

    # Second arrow
    p1 = samples[-1]
    p2 = samples[-2]
    angle = math.atan2(p1[1]-p2[1], p1[0]-p2[0])
    plot_arrow(p1[0], p1[1], angle)
    plot_arrow(point2[0], point2[1], angle, 0.3)
    plt.plot(p1[0], p1[1], "k.")

    if not save_figures:
        plt.pause(wait_time)

    if save_figures:
        global image_idx
        filename = "images/{:05d}.png".format(image_idx)
        image_idx = image_idx + 1
        print(filename)
        plt.savefig(filename)

image_idx = 0

##################################################
# Basic Dubins maneuver 
##################################################
if DUB:
    for t2 in np.arange(0, 2*math.pi, step):
        start = [point1[0], point1[1], 1]
        final = [point2[0], point2[1], t2]
        dubins_path = dubins.shortest_path(start, final, turning_radius)
        samples, _ = dubins_path.sample_many(step_size)

        plot_figure(samples, point1, point2, "Dubins maneuver")

##################################################
# Dubins Interval Problem (DIP)
##################################################
if DIP:
    sensing_radius = 0.0

    for t2 in np.arange(0, 2*math.pi, step):
        interval2 = [2 + t2, 0.5]
        dubins_path = dubins.shortest_path_DIP(point1, interval1, point2, interval2, turning_radius)
        samples, _ = dubins_path.sample_many(step_size)

        plot_figure(samples, point1, point2, "Dubins Interval Problem (DIP)", sensing_radius, interval1, interval2)

##################################################
# Generalized Dubins Interval Problem (GDIP)
##################################################
if GDIP:
    sensing_radius = sensing_radius_gdip

    for t2 in np.arange(0, 2*math.pi, step):
        interval2 = (2 + t2, 0.5)
        step_size = 0.01 * turning_radius
        dubins_path = dubins.shortest_path_GDIP(point1, interval1, sensing_radius, point2, interval2, sensing_radius, turning_radius)
        samples, _ = dubins_path.sample_many(step_size)

        plot_figure(samples, point1, point2, "Generalized Dubins Interval Problem (GDIP)", sensing_radius, interval1, interval2)

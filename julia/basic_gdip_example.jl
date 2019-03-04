#!/usr/bin/env julia

# Test for Wrapper for GDIP library libGDIP.so

##################################################
# Imports
##################################################

using Printf

# import PyPlot in version 2.8.0+
import Pkg
try
    using PyPlot
catch
    Pkg.add("PyPlot")
    using PyPlot
end
if Pkg.installed()["PyPlot"] < v"2.8.0"
    error("Please upgrade PyPlot to version 2.8.0+")
    exit(-1)
end
Circle = matplotlib.patches.Circle

# import wrapper for the GDIP solution
include("GDIP.jl")

##################################################
# Settings
##################################################

# Turning radius
turning_radius = 1.0
# Step of the theta2 angle in the visualization
step = 2 * pi / 100
# Wait time for the animation
wait_time = 0.01
# Save figures into a directory "images"
save_figures = false
if length(ARGS) > 0
    if "-save" == ARGS[1]
        save_figures = true
    end
end
if save_figures
    mkpath("images")
end
# Sensing radius in case of GDIP solution
sensing_radius_gdip = 0.5

# End points
point1 = [0.0, 0.0]
point2 = [5.0, 0.0]

# First angle interval (the second is changed)
interval1 = [1.0, 0.5]

# Enable only specific parts
DUB = false
DIP = false
GDIP = true 

#matplotlib.rcParams['text.usetex'] = true

##################################################
# Functions
##################################################

function plot_points(points; specs = "b", zorder = 1)
    x_val = [x[1] for x in points]
    y_val = [x[2] for x in points]
    return plt.plot(x_val, y_val, specs, zorder = zorder)
end

function plot_interval(point, interval, sensing_radius)
    ax = plt.gca()
    circle = Circle(point, sensing_radius, facecolor="yellow", edgecolor="orange", linewidth=1, alpha=0.2)
    ax.add_patch(circle)

    for angle in [interval[1], interval[1] + interval[2]]
        last = point + turning_radius * [cos(angle), sin(angle)]
        points = [point, last]
        plot_points(points, specs = "y-", zorder = 1)
    end
end

function plot_arrow(px, py, angle; alpha = 1)
    rad = 0.7 * turning_radius
    dx = rad * cos(angle)
    dy = rad * sin(angle)
    plt.arrow(px, py, dx, dy, head_width=0.05, head_length=0.1, fc="k", ec="k", zorder=2, alpha=alpha)
    plt.plot([px, px + dx], [py, py + dy], "k", visible=false)
end

function plot_figure(samples, point1, point2, title; sensing_radius = 0, interval1 = nothing, interval2 = nothing)
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
    if interval1 != nothing
        plot_interval(point1, interval1, sensing_radius)
    end
    if interval2 != nothing
        plot_interval(point2, interval2, sensing_radius)
    end

    # Plot samples of the Dubins maneuver
    ps = plot_points(samples)
    plt.legend([ps[1]], ["Dubins maneuver"])

    # First arrow
    p1 = samples[1]
    p2 = samples[2]
    angle = atan(p2[2]-p1[2], p2[1]-p1[1])
    plot_arrow(p1[1], p1[2], angle)
    plot_arrow(point1[1], point1[2], angle, alpha = 0.3)
    plt.plot(p1[1], p1[2], "k.")

    # Second arrow
    p1 = samples[length(samples)]
    p2 = samples[length(samples)-1]
    angle = atan(p1[2]-p2[2], p1[1]-p2[1])
    plot_arrow(p1[1], p1[2], angle)
    plot_arrow(point2[1], point2[2], angle, alpha = 0.3)
    plt.plot(p1[1], p1[2], "k.")

    if ! save_figures
        plt.pause(wait_time)
    end

    if save_figures
        filename = @sprintf("images/%05d.png", image_idx)
        #filename = "images/{:05d}.png".format(image_idx)
        global image_idx = image_idx + 1
        print(filename * "\n")
        plt.savefig(filename)
    end
end

image_idx = 0

##################################################
# Basic Dubins maneuver 
##################################################
if DUB
    for t2 in 0:step:(2*pi)
        start = [point1[1], point1[2], 1.0]
        final = [point2[1], point2[2], t2]
        gdip_init_dubins_maneuver(start, final, 1.0)
        samples = gdip_sample_many(step_size)

        plot_figure(samples, point1, point2, "Dubins maneuver")
    end
end

##################################################
# Dubins Interval Problem (DIP)
##################################################
if DIP
    sensing_radius = 0.0

    for t2 in 0:step:(2*pi)
        interval2 = [2 + t2, 0.5]
        gdip_init_dip(point1, interval1, point2, interval2, turning_radius)
        samples = gdip_sample_many(0.1)
        
        plot_figure(samples, point1, point2, "Dubins Interval Problem (DIP)", 
            sensing_radius = sensing_radius, interval1 = interval1, interval2 = interval2)

    end
end

##################################################
# Generalized Dubins Interval Problem (GDIP)
##################################################
if GDIP
    sensing_radius = sensing_radius_gdip

    for t2 in 0:step:(2*pi)
        interval2 = [2 + t2, 0.5]

        gdip_init_gdip(point1, interval1, sensing_radius, point2, interval2, sensing_radius, turning_radius)
        samples = gdip_sample_many(0.1)

        plot_figure(samples, point1, point2, "Dubins Interval Problem (DIP)", 
            sensing_radius = sensing_radius, interval1 = interval1, interval2 = interval2)

    end
end

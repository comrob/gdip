#!/usr/bin/env python3

import sys, os, re, math, copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch

# import the Python wrapper
from dubinswrapper import DubinsWrapper as dubins

################################################
# Scenarios
################################################
#define planning problems:
    #  map file 
    #  minimum turning radius
    #  sensing radius
    #  solver type
scenarios = [
    ("./problems/gdip-n10.txt", 1, 1, 'DTSPN-GDIP'),
    #("./problems/gdip-n10.txt", 0.5, 1, 'DTSPN-GDIP'),
    #("./problems/gdip-n10.txt", 1, 0.5, 'DTSPN-GDIP'),
]

################################################
# Settings
################################################

visualize = False
show = False

# Save figures into a directory "images"
save_figures = False
if len(sys.argv) > 1:
    if "-save" in sys.argv[1]:
        visualize = True
        save_figures = True
if save_figures:
    os.makedirs("images", exist_ok=True)

##################################################
# Functions
##################################################

def load_map(filename):
    """Read config with goal positions"""
    goals = []
    with open(filename) as fp:
        for line in fp:
            label, x, y = line.split()
            goals.append((float(x), float(y))) 
    return goals

def plot_points(points, specs = 'b'):
    x_val = [x[0] for x in points]
    y_val = [x[1] for x in points]
    plt.plot(x_val, y_val, specs)	  

def plot_circle(xy, radius):
    ax = plt.gca()
    circle = Circle(xy, radius, facecolor='yellow',edgecolor="orange", linewidth=1, alpha=0.2)
    ax.add_patch(circle)

def dist_euclidean(coord1, coord2):
    (x1, y1) = coord1
    (x2, y2) = coord2
    (dx, dy) = (x2 - x1, y2 - y1)
    return math.sqrt(dx * dx + dy * dy)

# cache results from computing length of the GDIP
lowerPathGDIPLenCache = {}

def lowerPathGDIP(s1, s2, turning_radius, cached = False):
    """Compute lower-bound path using GDIP between two configurations
    
    Arguments:  
        s1 - start; s2 - end; turning_radius
        cached - compute only distance and cache the results
    Returns:    
        Dubins maneuver (DubinsWrapper)
    """
    if cached:
        key = (s1, s2)
        if key in lowerPathGDIPLenCache:
            length = lowerPathGDIPLenCache[key]
            return (None, length)

    interval1 = [s1.alpha1, s1.alpha2 - s1.alpha1]
    interval2 = [s2.alpha1, s2.alpha2 - s2.alpha1]
    path = dubins.shortest_path_GDIP(s1.center, interval1, s1.radius, s2.center, interval2, s2.radius, turning_radius)
    length = path.get_length()
    
    if cached:
        lowerPathGDIPLenCache[key] = length 
        return (None, length)
    else:
        return (path, length)

def upperPathGDIP(s1, s2, turning_radius, cached = False):
    """Compute feasible Dubins path two configurations
    
    Arguments:  
        s1 - start; s2 - end; turning_radius
    Returns:    
        (Dubins maneuver 'DubinsWrapper', length)
    """
    q1 = s1.getFeasibleState()
    q2 = s2.getFeasibleState()
    dubins_path = dubins.shortest_path(q1, q2, turning_radius)
    return (dubins_path, dubins_path.get_length())

def compute_distances(samples, dst_fce, turning_radius = 0):
    n = len(samples)
    distances = []
        
    for i in range(n):
        ss1 = samples[i]
        ss2 = samples[(i+1) % n]

        n1 = len(ss1)
        n2 = len(ss2)

        sh = np.full((n1, n2), np.inf)
        for i1 in range(n1):
            for i2 in range(n2):
                dist = dst_fce(ss1[i1], ss2[i2], turning_radius, cached=True)
                sh[i1][i2] = dist[1]
        distances.append(sh)

    return distances

def find_shortest_tour(distances):
    n = len(distances)
    best_len = math.inf
    best_tour = []

    # maximal number of samples
    k_max = max([len(x) for x in distances])

    no_start = len(distances[0])
    for start in range(no_start):
        # shortest sh[region_idx][sample_idx]
        # contains (prev, length)
        sh = np.full((n+1, k_max), np.inf)
        # used edge
        prev = np.full((n+1, k_max), -1)

        sh[0,start] = 0

        for region_idx in range(n):
            n1 = len(distances[region_idx])
            n2 = len(distances[region_idx][0])
            for idx2 in range(n2):
                dst = sh[region_idx][:n1] + distances[region_idx][:,idx2]
                sh[region_idx+1][idx2] = np.min(dst)
                prev[region_idx+1][idx2]= np.argmin(dst)

        act_sol = sh[-1][start]
        if act_sol < best_len:
            best_len = act_sol
            tour = []
            act = start
            for i in range(n):
                act = prev[n-i][act]
                tour.append(act)
            best_tour = list(reversed(tour))

    return best_tour

def retrieve_path(samples, dst_fce, turning_radius, selected_samples):
    n = len(samples)
    path = []
    for a in range(0,n):
        g1 = samples[a][selected_samples[a]]
        g2 = samples[(a+1) % n][selected_samples[(a+1) % n]]
        path.append(dst_fce(g1, g2, turning_radius))
    return path

def path_len(path):
    return sum_list([dub.get_length() for dub in path])

def plot_path(path, turning_radius, settings):
    step_size = 0.01 * turning_radius
    for dub in path:
        configurations, _ = dub.sample_many(step_size)
        plot_points(configurations, settings) 

##################################################
# Target region given by the location and its sensing radius
##################################################
class TargetRegion:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def get_position_at_boundary(self, beta):
        return self.center + self.radius * np.array([math.cos(beta), math.sin(beta)])

##################################################
# Sample on the given target region
##################################################
class Sample:
    def __init__(self, targetRegion):
        # reference to the specific target region of the sample
        self.target = targetRegion

        # heading angle interval
        self.alpha1 = 0
        self.alpha2 = 2 * math.pi
        self.alphaResolution = 1
        # position (on the boundary) interval
        self.beta1 = 0
        self.beta2 = 2 * math.pi
        self.betaResolution = 1

        # center and radius of the position neighborhood on the boundary of the target region
        self.center = np.array(targetRegion.center)
        self.radius = targetRegion.radius

    def split(self, resolution):
        """Split the actual sample into two new ones.
        The first is stored directly in the actual sample, and the second is returned.
        If the required resolution is already met, then nothing is done and None is returned.

        Parameters:
            resolution: the requred resolution
        Returns:
            Sample - the second new sample
        """
        # prefer splitting according position resolution
        if self.betaResolution < resolution:
            sam1 = copy.copy(self)
            sam2 = copy.copy(self)
            sam1.betaResolution = sam2.betaResolution = 2 * self.betaResolution
            sam1.beta2 = sam2.beta1 = (self.beta1 + self.beta2) / 2
            sam1.update_center_radius()
            sam2.update_center_radius()
            return [sam1, sam2]
        if self.alphaResolution < resolution:
            sam1 = copy.copy(self)
            sam2 = copy.copy(self)
            sam1.alphaResolution = sam2.alphaResolution = 2 * self.alphaResolution
            sam1.alpha2 = sam2.alpha1 = (self.alpha1 + self.alpha2) / 2
            return [sam1, sam2]
        return None

    def update_center_radius(self):
        p1 = self.target.get_position_at_boundary(self.beta1)
        p2 = self.target.get_position_at_boundary(self.beta2)
        self.center = (p1 + p2) / 2
        self.radius = dist_euclidean(p1, p2) / 2

    def getFeasibleState(self):
        pos = self.target.get_position_at_boundary(self.beta1)
        q = np.zeros(3)
        q[0:2] = pos
        q[2] = self.alpha1
        return q

    def plot(self):
        ax = plt.gca()
        circle = Circle(self.center, self.radius, facecolor=None ,edgecolor="green", linewidth=1, alpha=0.2)
        ax.add_patch(circle)

##################################################
# Sampling structure which holds all the used samples
##################################################
class Sampling:
    def __init__(self, centers, sensingRadius):
        self.targets = [TargetRegion(c, sensingRadius) for c in centers]
        self.samples = [[Sample(t)] for t in self.targets]

    def refine_samples(self, selected, resolution):
        """Refine the seleted samples if the required resolution is not met.

        Parameters:
            slected: indexes of the selected samples (vector 1 x n)
            resolution: the requred resolution
        Returns:
            boolean - true if any sample is refined
        """
        n = len(self.samples)
        refined = False
        for i in range(n):
            to_split = selected[i]
            samp = self.samples[i][to_split]
            res = samp.split(resolution)
            if not res is None:
                self.samples[i][to_split] = res[0]
                self.samples[i].append(res[1])
                refined = True 
        return refined

##################################################
# The main solver class
##################################################
class GDIPSolver:
    def __init__(self, turning_radius, goals, sensing_radius):
        self.turning_radius = turning_radius
        self.sensing_radius = sensing_radius
        self.goals = goals
        self.sampling = Sampling(goals, sensing_radius)

        self.lower_path = []
        self.upper_path = []

        self.lower_bound = 0
        self.upper_bound = math.inf
    
    def plot_map(self):
        plt.clf()
        plt.axis('equal')
        plot_points(self.goals, 'ro')
        if self.sensing_radius != None:
            for goal in self.goals:
                plot_circle(goal, self.sensing_radius)

    def plot_tour_and_return_length(self, selected_samples, maneuver_function, color):
        sampling = self.sampling
        n = len(self.sampling.samples)
        step_size = 0.01 * self.turning_radius
        length = 0
        for a in range(0,n):
            g1 = sampling.samples[a][selected_samples[a]]
            g2 = sampling.samples[(a+1) % n][selected_samples[(a+1) % n]]

            path = maneuver_function(g1, g2, self.turning_radius)
            length += path[1]
            configurations, _ = path[0].sample_many(step_size)
            if visualize:
                plot_points(configurations, color)
        return length

    def plot_actual_and_return_bounds(self):
        """Plot the actual sampling, lower and upper bound path

        Returns:
            (double, double) - lower bound, upper bound
        """
        if visualize:
            self.plot_map()

        for s in self.sampling.samples:
            for ss in s:
                ss.plot()

        lower_selected_samples = self.find_lower_bound_tour()
        upper_selected_samples = self.find_upper_bound_tour()

        lower_bound = self.plot_tour_and_return_length(lower_selected_samples, lowerPathGDIP, 'r-')
        upper_bound = self.plot_tour_and_return_length(upper_selected_samples, upperPathGDIP, 'b-')
        return (lower_bound, upper_bound)
    
    def find_lower_bound_tour(self):
        """Select the samples which represent the shortest lower bound tour

        Returns:
            indexes of the samples (vector 1 x n)
        """
        distances = compute_distances(self.sampling.samples, lowerPathGDIP, turning_radius = self.turning_radius)
        selected_samples = find_shortest_tour(distances)
        return selected_samples    

    def find_upper_bound_tour(self):
        """Select the samples which represent the shortest upper bound (feasible) tour

        Returns:
            indexes of the samples (vector 1 x n)
        """
        distances = compute_distances(self.sampling.samples, upperPathGDIP, turning_radius = self.turning_radius)
        selected_samples = find_shortest_tour(distances)
        return selected_samples 


##################################################
# Main loop over selected scenarios
##################################################
for scenario in scenarios:
    # Load the problem and scenario settings
    filename = scenario[0]
    goals = load_map(filename)
    turning_radius = scenario[1]
    sensing_radius = scenario[2]
    solver_type = scenario[3]

    #tour planning part
    solver = GDIPSolver(turning_radius, goals, sensing_radius)
    solver.plot_actual_and_return_bounds()

    print("\n--- Problem: {}  Turning radius: {:6.2f}  Sensing radius: {:6.2f}  ---"
            .format(filename, turning_radius, sensing_radius))

    if show:
        plt.pause(0.1)

    max_resolution = 64
    act_res = 4
    while act_res <= max_resolution:
        refined = True
        while refined:
            selected_samples = solver.find_lower_bound_tour()
            refined = solver.sampling.refine_samples(selected_samples, act_res)
        (lower_bound, upper_bound) = solver.plot_actual_and_return_bounds()
        gap = (upper_bound - lower_bound) / upper_bound * 100.0
        print("Res: {:4d}  Lower: {:6.2f}  Upper: {:6.2f}  Gap(%): {:6.2f}"
            .format(act_res, lower_bound, upper_bound, gap))

        if visualize:
            plt.title("Maximum resolution: {:4d}".format(act_res))
        if show:
            plt.pause(0.1)
        if save_figures:
            plt.savefig("images/dtrp-res-{:04d}.png".format(act_res))
        
        act_res *= 2

    if show:
        plt.pause(2)


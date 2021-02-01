#!/usr/bin/env julia

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
if PyPlot.version < v"2.8.0"
    error("Please upgrade PyPlot to version 2.8.0+")
end
Circle = matplotlib.patches.Circle

try
    using TimerOutputs
catch
    Pkg.add("TimerOutputs")
    using TimerOutputs
end
# Create the timer object
to = TimerOutput()

# import wrapper for the GDIP solution
include("GDIP.jl")

################################################
# Scenarios
################################################
#define planning problems:
    #  map file 
    #  minimum turning radius
    #  sensing radius
    #  solver type
scenarios = [
    ("./problems/gdip-n10.txt", 1.0, 1.0, "DTSPN-GDIP"),
    #("./problems/gdip-n10.txt", 1.0, 0.5, "DTSPN-GDIP"),
    #("./problems/gdip-n10.txt", 0.5, 1.0, "DTSPN-GDIP"),
]

################################################
# Settings
################################################

visualize = true
show = true

# Save figures into a directory "images"
save_figures = false
if length(ARGS) > 0
    if "-save" == ARGS[1]
        visualize = true
        save_figures = true
    end
end
if save_figures
    mkpath("images")
end

##################################################
# Functions
##################################################

function load_map(filename)
    """Read config with goal positions"""
    goals = []
    lines = readlines(open(filename))
    for line in lines
        label, x, y = split(line)
        push!(goals, [parse(Float64,x), parse(Float64,y)]) 
    end
    return goals
end

function plot_points(points; specs = "b")
    x_val = [x[1] for x in points]
    y_val = [x[2] for x in points]
    plt.plot(x_val, y_val, specs)	 
end 

function plot_circle(xy, radius)
    ax = plt.gca()
    circle = Circle(xy, radius, facecolor="yellow",edgecolor="orange", linewidth=1, alpha=0.2)
    ax.add_patch(circle)
end

function dist_euclidean(coord1, coord2)
    (x1, y1) = coord1
    (x2, y2) = coord2
    (dx, dy) = (x2 - x1, y2 - y1)
    return sqrt(dx * dx + dy * dy)
end


# cache results from computing length of the GDIP
lowerPathGDIPLenCache = Dict()

function lowerPathGDIP(s1, s2, turning_radius; cached = false)
    """Compute lower-bound path using GDIP between two configurations
    
    Arguments:  
        s1 - start; s2 - end; turning_radius
        cached - compute only distance and cache the results
    Returns:    
        Dubins maneuver (DubinsWrapper)
    """
    #if cached:
    #    key = (s1, s2)
    #    if key in lowerPathGDIPLenCache:
    #        length = lowerPathGDIPLenCache[key]
    #        return (None, length)

    interval1 = [s1.alpha1, s1.alpha2 - s1.alpha1]
    interval2 = [s2.alpha1, s2.alpha2 - s2.alpha1]
    #@timeit to "gdip" 
    return gdip_init_gdip(s1.center, interval1, s1.radius, s2.center, interval2, s2.radius, turning_radius)
    
    #if cached:
    #    lowerPathGDIPLenCache[key] = length 
    #    return (None, length)
    #else:
    #    return (path, length)
    return length
end

function upperPathGDIP(s1, s2, turning_radius; cached = false)
    """Compute feasible Dubins path two configurations
    
    Arguments:  
        s1 - start; s2 - end; turning_radius
    Returns:    
        (Dubins maneuver 'DubinsWrapper', length)
    """
    q1 = getFeasibleState(s1)
    q2 = getFeasibleState(s2)
    return gdip_init_dubins_maneuver(q1, q2, turning_radius)
end

function compute_distances(samples, dst_fce; turning_radius = 0)
    n = length(samples)
    distances = []
        
    for i in 1:n
        ss1 = samples[i]
        ss2 = samples[(i % n) + 1]

        n1 = length(ss1)
        n2 = length(ss2)

        sh = fill(Inf, (n1, n2))
        for i1 in 1:n1
            for i2 in 1:n2
                sh[i1,i2] = dst_fce(ss1[i1], ss2[i2], turning_radius, cached=true)
            end
        end
        push!(distances, sh)
    end

    return distances
end

function update_distances(samples, distances, selected_samples, dst_fce; turning_radius = 0)
    n = length(samples)    
    for i in 1:n
        ss1 = samples[i]
        ss2 = samples[(i % n) + 1]

        n1 = length(ss1)
        n2 = length(ss2)

        sh_old = distances[i]
        siz_old = size(sh_old) 
        sh = fill(NaN, (n1, n2))
        sh[1:siz_old[1],1:siz_old[2]] = sh_old

        #@show size(sh), selected_samples[i], sh[selected_samples[i], :]
        sh[selected_samples[i], :] *= NaN 
        sh[:,selected_samples[(i%n)+1]] *= NaN

        # Alternative without going throw the whole distance matrix
        #=
        for j in 1:n1
            dist = dst_fce(ss1[j], ss2[n2], turning_radius, cached=true)[1]
            sh[j,n2] = dist
        end

        for j in 1:n2
            dist = dst_fce(ss1[n1], ss2[j], turning_radius, cached=true)[1]
            sh[n1,j] = dist
        end

        i1 = selected_samples[i]
        for i2 in 1:n2
            dist = dst_fce(ss1[i1], ss2[i2], turning_radius, cached=true)[1]
            sh[i1, i2] = dist
        end

        i2 = selected_samples[(i%n)+1]
        for i1 in 1:n1
            dist = dst_fce(ss1[i1], ss2[i2], turning_radius, cached=true)[1]
            sh[i1, i2] = dist
        end
        =#

        for i1 in 1:n1
            for i2 in 1:n2
                dist = sh[i1,i2]
                if isnan(dist)
                    @timeit to "dist_fce" begin
                        sh[i1,i2] = dst_fce(ss1[i1], ss2[i2], turning_radius, cached=true)
                    end
                end
            end
        end
        
        distances[i] = sh
    end

    return distances
end

function find_shortest_tour(distances)
    n = size(distances)[1]
    best_len = Inf
    best_tour = []

    # TODO - remove
    #global dist = distances

    # maximal number of samples
    k_max = maximum([size(x)[1] for x in distances])

    no_start = size(distances[1])[1]
    #@show no_start, k_max, n
    for start in 1:no_start
        # shortest sh[region_idx][sample_idx]
        sh = fill(Inf, (n+1, k_max))
        # used edge
        prev = fill(-1, (n+1, k_max))

        sh[1,start] = 0

        for region_idx in 1:n
            (n1, n2) = size(distances[region_idx])
            #@show n1, n2, distances[region_idx]
            for idx2 in 1:n2
                #@show distances[region_idx]
                #@show sh[region_idx,1:n1], distances[region_idx][:,idx2]
                dst = sh[region_idx,1:n1] + distances[region_idx][:,idx2]
                sh[region_idx+1,idx2] = minimum(dst)
                #@show dst
                #@show minimum(dst)
                prev[region_idx+1,idx2]= argmin(dst)
            end
        end

        #@show size(sh), n, start
        act_sol = sh[n+1,start]
        if act_sol < best_len
            best_len = act_sol
            tour = []
            act = start
            for i in 1:n
                act = prev[n-i+2,act]
                push!(tour, act)
            end
            best_tour = reverse(tour)
            #print(best_tour)
        end
    end

    return best_tour
end

function retrieve_path(samples, dst_fce, turning_radius, selected_samples)
    n = length(samples)
    path = []
    for a in 1:n
        g1 = samples[a][selected_samples[a]]
        g2 = samples[(a+1) % n][selected_samples[(a+1) % n]]
        path.append(dst_fce(g1, g2, turning_radius))
    end
    return path
end

function path_len(path)
    return sum_list([dub.get_length() for dub in path])
end

function plot_path(path, turning_radius, settings)
    step_size = 0.01 * turning_radius
    for dub in path
        configurations, _ = dub.sample_many(step_size)
        plot_points(configurations, settings) 
    end
end

##################################################
# Target region given by the location and its sensing radius
##################################################
mutable struct TargetRegion
    center
    radius
end

function get_position_at_boundary(region::TargetRegion, beta)
    return region.center + region.radius * [cos(beta), sin(beta)]
end

##################################################
# Sample on the given target region
##################################################
mutable struct Sample
    target

    center
    radius

    alpha1
    alpha2
    alphaResolution

    beta1
    beta2
    betaResolution
end

Base.copy(s::Sample) = Sample(s.target, s.center, s.radius, s.alpha1, s.alpha2, s.alphaResolution, s.beta1, s.beta2, s.betaResolution)

function Sample_init(targetRegion)
    alpha1 = 0
    alpha2 = 2 * pi
    alphaResolution = 1

    beta1 = 0
    beta2 = 2 * pi
    betaResolution = 1

    return Sample(targetRegion, targetRegion.center, targetRegion.radius, alpha1, alpha2, alphaResolution, beta1, beta2, betaResolution)
end

# TODO missing functions

function Sample_plot(sample)
    ax = plt.gca()
    circle = Circle(sample.center, sample.radius, facecolor=nothing ,edgecolor="green", linewidth=1, alpha=0.2)
    ax.add_patch(circle)
end

function Sample_split(sample, resolution)
    """Split the actual sample into two new ones.
    The first is stored directly in the actual sample, and the second is returned.
    If the required resolution is already met, then nothing is done and None is returned.

    Parameters:
        resolution: the requred resolution
    Returns:
        Sample - the second new sample
    """
    # prefer splitting according position resolution
    if sample.betaResolution < resolution
        sam1 = copy(sample)
        sam2 = copy(sample)
        sam1.betaResolution = sam2.betaResolution = 2 * sample.betaResolution
        sam1.beta2 = sam2.beta1 = (sample.beta1 + sample.beta2) / 2
        update_center_radius(sam1)
        update_center_radius(sam2)
        return [sam1, sam2]
    end
    if sample.alphaResolution < resolution
        sam1 = copy(sample)
        sam2 = copy(sample)
        sam1.alphaResolution = sam2.alphaResolution = 2 * sample.alphaResolution
        sam1.alpha2 = sam2.alpha1 = (sample.alpha1 + sample.alpha2) / 2
        return [sam1, sam2]
    end
    return nothing
end

function update_center_radius(self)
    p1 = get_position_at_boundary(self.target, self.beta1)
    p2 = get_position_at_boundary(self.target, self.beta2)
    self.center = (p1 + p2) / 2
    self.radius = dist_euclidean(p1, p2) / 2
end

function getFeasibleState(self)
    pos = get_position_at_boundary(self.target, self.beta1)
    q = zeros(3)
    q[1:2] = pos
    q[3] = self.alpha1
    return q
end

##################################################
# Sampling structure which holds all the used samples
##################################################
mutable struct Sampling
    targets
    samples
end

function Sampling_init(centers, sensingRadius)
    targets = [TargetRegion(c, sensingRadius) for c in centers]
    samples = [[Sample_init(t)] for t in targets]
    return Sampling(targets, samples)
end

function refine_samples(self, selected, resolution)
    """Refine the seleted samples if the required resolution is not met.

    Parameters:
        slected: indexes of the selected samples (vector 1 x n)
        resolution: the requred resolution
    Returns:
        boolean - true if any sample is refined
    """
    n = length(self.samples)
    refined = false
    for i in 1:n
        to_split = selected[i]
        samp = self.samples[i][to_split]
        res = Sample_split(samp, resolution)
        if res != nothing
            self.samples[i][to_split] = res[1]
            push!(self.samples[i], res[2])
            refined = true 
        end
    end
    return refined
end

##################################################
# The main solver class
##################################################
mutable struct GDIPSolver
    turning_radius::Float64
    sensing_radius::Float64

    sampling::Sampling

    goals

    lower_path::Array{Integer,1}
    upper_path::Array{Integer,1}

    lower_bound::Float64
    upper_bound::Float64
end

function GDIPSolver_init(turning_radius, goals, sensing_radius)
    return GDIPSolver(turning_radius, sensing_radius, Sampling_init(goals, sensing_radius), goals, [], [], 0, Inf)
end

function GDIPSolver_plot_map(solver)
    plt.clf()
    plt.axis("equal")
    plot_points(solver.goals, specs = "ro")
    if solver.sensing_radius != 0
        for goal in solver.goals
            plot_circle(goal, solver.sensing_radius)
        end
    end
end

function GDIPSolver_plot_tour_and_return_length(solver, selected_samples, maneuver_function, color)
    sampling = solver.sampling
    n = length(solver.sampling.samples)
    step_size = 0.01 * solver.turning_radius
    len = 0
    for a in 1:n
        b = (a%n)+1
        g1 = sampling.samples[a][selected_samples[a]]
        g2 = sampling.samples[b][selected_samples[b]]

        len += maneuver_function(g1, g2, solver.turning_radius)
        configurations = gdip_sample_many(step_size)
        if visualize
            plot_points(configurations, specs = color)
        end
    end
    return len
end

function GDIPSolver_plot_actual_and_return_bounds(solver)
    """Plot the actual sampling, lower and upper bound path

    Returns:
        (double, double) - lower bound, upper bound
    """
    if visualize
        GDIPSolver_plot_map(solver)
    end

    for s in solver.sampling.samples
        for ss in s
            Sample_plot(ss)
        end
    end

    lower_selected_samples = GDIPSolver_find_shortest_tour(solver, lowerPathGDIP)[1]
    upper_selected_samples = GDIPSolver_find_shortest_tour(solver, upperPathGDIP)[1]

    lower_bound = GDIPSolver_plot_tour_and_return_length(solver, lower_selected_samples, lowerPathGDIP, "r-")
    upper_bound = GDIPSolver_plot_tour_and_return_length(solver, upper_selected_samples, upperPathGDIP, "b-")
    return (lower_bound, upper_bound)
end

function GDIPSolver_find_shortest_tour(solver, fce; dist = nothing, selected = nothing)
    """Select the samples which represent the shortest lower bound tour

    Returns:
        indexes of the samples (vector 1 x n)
    """
    if dist == nothing || selected == nothing
        distances = compute_distances(solver.sampling.samples, fce, turning_radius = solver.turning_radius)
    else
        @timeit to "Update" distances = update_distances(solver.sampling.samples, dist, selected, fce, turning_radius = solver.turning_radius)
    end
    @timeit to "Find" selected_samples = find_shortest_tour(distances)
    return selected_samples, distances
end

##################################################
# Main loop over selected scenarios
##################################################
for scenario in scenarios
    # Load the problem and scenario settings
    filename = scenario[1]
    goals = load_map(filename)
    turning_radius = scenario[2]
    sensing_radius = scenario[3]
    solver_type = scenario[4]

    #tour planning part
    solver = GDIPSolver_init(turning_radius, goals, sensing_radius)
    GDIPSolver_plot_actual_and_return_bounds(solver)

    @printf("\n--- Problem: %s  Turning radius: %6.2f  Sensing radius: %6.2f  ---\n",
        filename, turning_radius, sensing_radius)

    if show
        plt.pause(0.1)
    end

    max_resolution = 64
    act_res = 4
    while act_res <= max_resolution
        refined = true
        distances = nothing
        selected_samples = nothing
        @timeit to "Iteration" begin
            while refined
                @timeit to "Tour" selected_samples, distances = GDIPSolver_find_shortest_tour(
                    solver, lowerPathGDIP, dist = distances, selected = selected_samples)
                @timeit to "Refine" refined = refine_samples(solver.sampling, selected_samples, act_res)
            end
        end
        (lower_bound, upper_bound) = GDIPSolver_plot_actual_and_return_bounds(solver)
        gap = (upper_bound - lower_bound) / upper_bound * 100.0
        @printf("Res: %4d  Lower: %6.2f  Upper: %6.2f  Gap(%%): %6.2f\n",
            act_res, lower_bound, upper_bound, gap)

        

        if visualize
            plt.title(@sprintf("Maximum resolution: %4d", act_res))
        end

        if show
            plt.pause(0.1)
        end

        if save_figures
            plt.savefig(@sprintf("images/dtrp-res-%04d.png", act_res))
        end
        
        act_res *= 2
    end

    @show to
    #show(to; allocations = false, sortby=:name)
    #show(to)
    print("\n")

    if show
        plt.pause(0.5)
    end
end

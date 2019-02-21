#!/usr/bin/env julia

# Wrapper for GDIP library libGDIP.so

using Libdl

# Open the library explicitly.
lib = Libdl.dlopen("../gdip/lib/libGDIP.so")

@inline function gdip_get_length()
    return ccall( (:julia_get_length, :libGDIP), Cdouble, () )
end

@inline function gdip_sample_many(step::Float64)
    length = gdip_get_length()
    samples = []
    sample_l = collect(0:step:length)
    push!(sample_l, length)
    for l in sample_l
        ccall( (:julia_sample_state_to_tmp, :libGDIP), Cvoid, (Cdouble,), l)
        x = ccall( (:julia_get_tmp_x, :libGDIP), Cdouble, (), )
        y = ccall( (:julia_get_tmp_y, :libGDIP), Cdouble, (), )
        theta = ccall( (:julia_get_tmp_theta, :libGDIP), Cdouble, (), )
        push!(samples, [x,y,theta])
    end
    return samples
end

@inline function gdip_init_dubins_maneuver(
    from::Array{Float64,1}, to::Array{Float64,1}, turningRadius::Float64
)
    length = ccall(
        (:julia_set_configurations, :libGDIP),                            # name of C function and library
        Cdouble,                                                          # output type
        (Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble),  # tuple of input types
        from[1], from[2], from[3], to[1], to[2], to[3], turningRadius     # names of Julia variables to pass in
    )
    return length
end

@inline function gdip_init_dip(
    p1::Array{Float64,1}, interval1::Array{Float64,1}, 
    p2::Array{Float64,1}, interval2::Array{Float64,1}, 
    turningRadius::Float64
)
    length = ccall( 
        (:julia_set_configurations_dip, :libGDIP),                                                         # name of C function and library
        Cdouble,                                                                                           # output type
        (Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble),                 # tuple of input types
        p1[1], p1[2], interval1[1], interval1[2], p2[1], p2[2], interval2[1], interval2[2], turningRadius  # names of Julia variables to pass in
    )
    return length
end

@inline function gdip_init_gdip(
    p1::Array{Float64,1}, interval1::Array{Float64,1}, radius1::Float64,
    p2::Array{Float64,1}, interval2::Array{Float64,1}, radius2::Float64,
    turningRadius::Float64
)
    length = ccall(
        (:julia_set_configurations_gdip, :libGDIP),                                                                          # name of C function and library
        Cdouble,                                                                                                             # output type
        (Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble),                 # tuple of input types
        p1[1], p1[2], interval1[1], interval1[2], radius1, p2[1], p2[2], interval2[1], interval2[2], radius2,  turningRadius # names of Julia variables to pass in
    )
    return length
end
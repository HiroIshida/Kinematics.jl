abstract type AbstractSDF end

function gradient!(sdf::AbstractSDF, p::StaticVector{3, <:AbstractFloat}, out_grad::AbstractVector)
    eps = 1e-7
    for i in 1:3
        sdf.tmp[:] = p
        sdf.tmp[i] += eps
        out_grad[i] = (sdf(sdf.tmp; do_cache=false) - sdf.val_cache)/eps
    end
end

mutable struct BoxSDF <: AbstractSDF
    pose::Transform
    inv_pose::Transform
    width::SVector3f

    val_cache::Float64
    tmp::MVector{3, Float64} # local variable
end
function BoxSDF(pose::Transform, width::SVector3f)
    BoxSDF(pose, inv(pose), width, 0.0, MVector3f(0, 0, 0))
end
function BoxSDF(width::SVector3f)
    BoxSDF(one(Transform), one(Transform), width)
end

function (sdf::BoxSDF)(p::StaticVector{3, <:AbstractFloat}; do_cache=true)
    half_extent = 0.5 * sdf.width
    p_sdfframe = sdf.inv_pose * p
    q = abs.(p_sdfframe) - half_extent
    dist = norm(max.(q, 0.0)) + min(maximum(q), 0.0)
    do_cache && (sdf.val_cache = dist)
    return dist
end

struct UnionSDF
    sdfs::Vector{AbstractSDF}
    vals_cache::Vector{Float64} # maybe used in the grad computation
end

function UnionSDF(sdfs::Vector{<:AbstractSDF})
    vals_cache = zeros(length(sdfs))
    UnionSDF(sdfs, vals_cache)
end

# Note that this Union sdf computation is inexact
# https://www.iquilezles.org/www/articles/interiordistance/interiordistance.htm
# However, noting that we have (sphere-radius + margin), this is not a big problem
function (this::UnionSDF)(p::StaticVector{3, <:AbstractFloat})
    for i in 1:length(this.sdfs)
        this.vals_cache[i] = this.sdfs[i](p)
    end
    return minimum(this.vals_cache)
end

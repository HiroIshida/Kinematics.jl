abstract type AbstractSDF end

struct BoxSDF <: AbstractSDF
    pose::Transform
    inv_pose::Transform
    width::SVector3f
end
function BoxSDF(pose::Transform, width::SVector3f)
    BoxSDF(pose, inv(pose), width)
end
function BoxSDF(width::SVector3f)
    BoxSDF(one(Transform), one(Transform), width)
end

function (sdf::BoxSDF)(p::StaticVector{3, <:AbstractFloat})
    half_extent = 0.5 * sdf.width
    p_sdfframe = sdf.inv_pose * p
    q = abs.(p_sdfframe) - half_extent
    return norm(max.(q, 0.0)) + min(maximum(q), 0.0)
end

struct UnionSDF
    sdfs::Vector{AbstractSDF}
    vals_cache::Vector{Float64} # maybe used in the grad computation
end

function UnionSDF(sdfs::Vector{<:AbstractSDF})
    vals_cache = zeros(length(sdfs))
    UnionSDF(sdfs, vals_cache)
end

function (this::UnionSDF)(p::StaticVector{3, <:AbstractFloat})
    for i in 1:length(this.sdfs)
        this.vals_cache[i] = this.sdfs[i](p)
    end
    return minimum(this.vals_cache)
end

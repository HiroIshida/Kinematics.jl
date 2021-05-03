mutable struct SdfLink <: LinkType end

abstract type AttachStyle end
struct IsAttached <: AttachStyle end
struct IsStandAlone <: AttachStyle end

abstract type AbstractSDF{AS<:AttachStyle} end

function inv_pose(sdf::AbstractSDF{IsAttached})
end


function gradient!(sdf::AbstractSDF, p::StaticVector{3, <:AbstractFloat}, out_grad::AbstractVector)
    eps = 1e-7
    for i in 1:3
        sdf.tmp[:] = p
        sdf.tmp[i] += eps
        out_grad[i] = (sdf(sdf.tmp; do_cache=false) - sdf.val_cache)/eps
    end
end

function attach_to_link(sdf::AbstractSDF{IsAttached}, mech::Mechanism, parent_link::Link)
    new_link = Link(SdfLink, sdf.name)
    add_new_link(mech, new_link, parent_link, sdf.pose)
    sdf.link = new_link
end

mutable struct BoxSDF{AS} <: AbstractSDF{AS}
    attach_style::AS
    name::String
    link::Union{Link, Nothing} # TODO should be member of IsAttached
    pose::Transform
    inv_pose::Transform
    width::SVector3f

    val_cache::Float64
    tmp::MVector{3, Float64} # local variable
end

function BoxSDF(
        pose::Transform, width::SVector3f; attach_style::AS=IsStandAlone()) where AS<:AttachStyle
    name = "boxsdf_" * string(UUIDs.uuid1())
    BoxSDF{AS}(attach_style, name, nothing, pose, inv(pose), width, 0.0, MVector3f(0, 0, 0))
end

function BoxSDF(boxmd::BoxMetaData; attach_style::AttachStyle=IsStandAlone())
    BoxSDF(boxmd.origin, boxmd.extents; attach_style=attach_style)
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
    min_idx_cache::Int
end

function UnionSDF(mech::Mechanism)
    sdfs = AbstractSDF[]
    for link in mech.links
        meta = link.geometric_meta_data
        if typeof(meta) == BoxMetaData
            as = IsAttached()
            sdf = BoxSDF(meta; attach_style=as)
            push!(sdfs, sdf)
            attach_to_link(sdf, mech, link)
        else
            @warn "currently only metadata type of box is available"
        end
    end
    UnionSDF(sdfs)
end

function UnionSDF(sdfs::Vector{<:AbstractSDF})
    vals_cache = zeros(length(sdfs))
    min_idx = 0
    UnionSDF(sdfs, vals_cache, min_idx)
end

# Note that this Union sdf computation is inexact
# https://www.iquilezles.org/www/articles/interiordistance/interiordistance.htm
# However, noting that we have (sphere-radius + margin), this is not a big problem
function (this::UnionSDF)(p::StaticVector{3, <:AbstractFloat})
    for i in 1:length(this.sdfs)
        this.vals_cache[i] = this.sdfs[i](p; do_cache=true)
    end
    this.min_idx_cache = argmin(this.vals_cache)
    return this.val_cache[this.min_idx_cache]
end

function gradient!(this::UnionSDF, p::StaticVector{3, <:AbstractFloat}, out_grad::AbstractVector)
    sdf_min = this.sdfs[this.min_idx_cache]
    gradient!(sdf_min, p, out_grad)
end

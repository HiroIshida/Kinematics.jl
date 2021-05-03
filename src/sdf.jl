abstract type AttachStyle end
struct IsStandAlone <: AttachStyle end
struct IsAttached <: AttachStyle
    link::Link
    mech::Mechanism
end

abstract type AbstractSDF{AS<:AttachStyle} end

mutable struct SdfLinkType <: LinkType
    sdf::Union{AbstractSDF{IsAttached}, Nothing}
end

function get_transform(m::Mechanism, link::Link{SdfLinkType})
    iscached(m, link) && (return get_cache(m, link))
    pose = _get_transform(m, link)
    link.sdf.inv_pose = inv(poes) # cache
    return _get_transform(m, link)
end

inv_pose(sdf::AbstractSDF) = sdf.inv_pose

function inv_pose(sdf::AbstractSDF{IsAttached})
    mech, link = sdf.attach_style.mech, sdf.attach_style.link
    # see get_transform(m::Mechanism, link::Link{SdfLinkType})
    inv_pose_cached = iscached(mech, link) # because inv_pose is cached simul. in get_transform 
    if ~inv_pose_cached
        get_transform(mech, link)
    end
    return sdf.inv_pose
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
    new_link = sdf.attach_style.link
    add_new_link(mech, new_link, parent_link, sdf.pose)
end

mutable struct BoxSDF{AS} <: AbstractSDF{AS}
    attach_style::AS
    pose::Transform
    inv_pose::Transform
    width::SVector3f

    val_cache::Float64
    tmp::MVector{3, Float64} # local variable
end

function BoxSDF(
        pose::Transform, width::SVector3f; attach_style::AS=IsStandAlone()) where AS<:AttachStyle
    BoxSDF{AS}(attach_style, pose, inv(pose), width, 0.0, MVector3f(0, 0, 0))
end

function BoxSDF(boxmd::BoxMetaData; attach_style::AttachStyle=IsStandAlone())
    BoxSDF(boxmd.origin, boxmd.extents; attach_style=attach_style)
end

function (sdf::BoxSDF)(p::StaticVector{3, <:AbstractFloat}; do_cache=true)
    half_extent = 0.5 * sdf.width
    p_sdfframe = inv_pose(sdf) * p
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
            new_link = Link(SdfLinkType(nothing), "boxsdf_" * string(UUIDs.uuid1()))
            sdf = BoxSDF(meta; attach_style=IsAttached(new_link, mech))
            new_link.link_type.sdf = sdf
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

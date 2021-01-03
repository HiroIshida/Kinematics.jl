module Kinematics

using PyCall
const __skrobot__ = PyCall.PyNULL()

using Rotations
using StaticArrays
const SVector3f = SVector{3, Float64}
const SVector4f = SVector{4, Float64}

macro debugassert(test)
  esc(:(if $(@__MODULE__).debugging()
    @assert($test)
   end))
end
debugging() = false

include("cache.jl")
include("transform.jl")
include("mechanism.jl")
include("load_urdf.jl")

export Transform

export CacheVector, invalidate!, set_cache!, iscached, get_cache

export Mechanism, Transform, parent_link, child_link, child_links, parent_joint, child_joints, find_link, find_joint, isroot, isleaf
export parse_urdf


function __init__()
    copy!(__skrobot__, pyimport("skrobot"))
end

end

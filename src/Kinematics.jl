module Kinematics

using PyCall
const __skrobot__ = PyCall.PyNULL()
const __trimesh__ = PyCall.PyNULL()

using Rotations
using StaticArrays
using LinearAlgebra

using MeshCat
using GeometryBasics
using CoordinateTransformations
using Colors: Color, Colorant, RGB, RGBA, alpha, hex, red, green, blue
using UUIDs
using NLopt

const SVector3f = SVector{3, Float64}
const SVector4f = SVector{4, Float64}

macro debugassert(test)
  esc(:(if $(@__MODULE__).debugging()
    @assert($test)
   end))
end
debugging() = false

include("cache.jl")
include("stack.jl")
include("transform.jl")
include("mechanism.jl")
include("load_urdf.jl")
include("algorithm.jl")
include("visual.jl")
include("collision.jl")

export Transform, rotation, translation
export CacheVector, invalidate_cache!, set_cache!, iscached, get_cache, extend!
export PseudoStack
export parse_urdf
export Mechanism, Transform, parent_link, child_link, child_links, parent_joint, child_joints, find_link, find_joint, isroot, isleaf, joint_angle, set_joint_angle, set_joint_angles, is_relevant, get_joint_angles!, add_new_link

# from algorithm.jl
export get_transform, get_jacobian, get_jacobian!, point_inverse_kinematics

# from visual.jl
export add_mechanism, update, add_frame

#from collision.jl
export collision_trimesh, compute_swept_sphere

export __skrobot__


function __init__()
    copy!(__skrobot__, pyimport("skrobot"))
    copy!(__trimesh__, pyimport("trimesh"))
end

end

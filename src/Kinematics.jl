module Kinematics

using PyCall
const __skrobot__ = PyCall.PyNULL()
const __trimesh__ = PyCall.PyNULL()
const __scipyopt__ = PyCall.PyNULL()

using Rotations
using StaticArrays
using SparseArrays
using LinearAlgebra

using MeshCat
using GeometryBasics
using CoordinateTransformations
using Colors: Color, Colorant, RGB, RGBA, alpha, hex, red, green, blue
using UUIDs
using NLopt
using Ipopt

const SVector3f = SVector{3, Float64}
const SVector4f = SVector{4, Float64}
const MVector3f = MVector{3, Float64}

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
include("sdf.jl")
include("visual.jl")
include("collision.jl")
include("planning.jl")
include("inverse_kinematics.jl")
include("models.jl")

export Transform, rotation, translation, rpy
export CacheVector, invalidate_cache!, set_cache!, iscached, get_cache, extend!
export PseudoStack
export parse_urdf
export Mechanism, Transform, parent_link, child_link, child_links, parent_joint, child_joints, find_link, find_joint, isroot, isleaf, joint_angle, set_joint_angle, set_joint_angles, is_relevant, get_joint_angles!, get_joint_angles, add_new_link, User, Link

# from algorithm.jl
export get_transform, get_jacobian, get_jacobian!, inverse_kinematics!

#from sdf.jl
export BoxSDF, UnionSDF

#from collision.jl
export SweptSphereCollisionChecker, collision_trimesh, compute_swept_sphere, add_coll_links, add_sscc, compute_coll_dists, compute_coll_dists!, compute_coll_dists_and_grads!, compute_coll_dists_and_grads

# from visual.jl
export add_mechanism, update, add_frame, to_affine_map, create_vis_sphere, add_sdf

# from planning.jl
export create_straight_trajectory, plan_trajectory, PoseConstraint, ConfigurationConstraint

# from models.jl
export load_pr2, rarm_joints, larm_joints, rarm_collision_links, larm_collision_links, reset_manip_pose

# other stuff
export __skrobot__


function __init__()
    copy!(__skrobot__, pyimport("skrobot"))
    copy!(__trimesh__, pyimport("trimesh"))
    try
        copy!(__scipyopt__, pyimport("scipy.optimize")) # optional
    catch e
        @warn "Couldn't import scipy"
    end
end

end

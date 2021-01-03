module Kinematics

using PyCall
const __skrobot__ = PyCall.PyNULL()

using Rotations
using StaticArrays
const SVector3f = SVector{3, Float64}


include("mechanism.jl")
include("load_urdf.jl")

export Mechanism, parent_link, child_link, child_links, parent_joint, child_joints, find_link, find_joint, isroot, isleaf
export parse_urdf


function __init__()
    copy!(__skrobot__, pyimport("skrobot"))
end

end

module Kinematics

using PyCall
const __skrobot__ = PyCall.PyNULL()

using Rotations
using StaticArrays
const SVector3f = SVector{3, Float64}


include("mechanism.jl")
include("load_urdf.jl")

export Mechanism, parse_urdf

function __init__()
    copy!(__skrobot__, pyimport("skrobot"))
end

end

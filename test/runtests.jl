using Kinematics
using Test 
using LinearAlgebra
using Rotations
using StaticArrays
import JSON
const SVector3f = SVector{3, Float64}
Kinematics.debugging() = true

include("test_datastructure.jl")
include("test_mechanism.jl")
include("test_kinematics.jl")
include("test_sdf.jl")
include("test_collision.jl")
include("test_planning.jl")
include("test_inverse_kinematics.jl")

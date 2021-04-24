using Kinematics
using Test 
using Rotations
using StaticArrays
import JSON
Kinematics.debugging() = true

include("test_datastructure.jl")
include("test_mechanism.jl")
include("test_kinematics.jl")
include("test_collision.jl")
include("test_planning.jl")

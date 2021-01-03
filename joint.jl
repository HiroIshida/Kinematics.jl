using Rotations: rotation_between, Rotation, RotMatrix, UnitQuaternion, RotXYZ
using StaticArrays

const SVector3f = SVector{3, Float64}

mutable struct Transform
    positoin::SVector3f
    rotation::UnitQuaternion
end

function Transform(position::SVector3f, rotmat::AbstractMatrix)
    rotation = UnitQuaternion(rotmat)
    return Transform(position, rotation)
end


abstract type JointType end
struct Revolute<:JointType
    axis::SVector3f
    lower_limit::Float64
    upper_limit::Float64
end
Revolute(axis, lower_limit, upper_limit) = (Revolute(SVector3f(axis), lower_limit, upper_limit))
Revolute(axis) = (Revolute(SVector3f(axis), -Inf, Inf))


struct Fixed<:JointType end

struct Joint{JT<:JointType}
    name::String
    id::Int
    plink_id::Int
    clink_id::Int
    pose::Transform
    jt::JT
end

function Joint(name, id, plink_id, clink_id, pos, rotmat, jt::JT) where {JT<:JointType}
    pos = SVector3f(pos)
    #rotmat = RotXYZ(rpy...)
    pose = Transform(pos, rotmat)
    Joint{JT}(name, id, plink_id, clink_id, pose, jt)
end

"""
rev = Revolute([0, 0, 0])
jt = Joint("hoge", 0, 0, 0, [1, 2, 3], [1, 2, 3], rev)
"""

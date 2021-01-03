using Rotations: rotation_between, Rotation, RotMatrix, UnitQuaternion, RotXYZ
using StaticArrays
using Lazy

const SVector3f = SVector{3, Float64}

mutable struct Transform
    positoin::SVector3f
    rotation::UnitQuaternion
end

function Transform(position::SVector3f, rotmat::AbstractMatrix)
    rotation = UnitQuaternion(rotmat)
    return Transform(position, rotation)
end

abstract type Joint end
getproperty(j::Joint)

struct JointCommonData
    id::Int
    plink_id::Int
    clink_id::Int
    pose::Transform
end

struct RevoluteJoint<:Joint
    common::JointCommonData
    axis::SVector3f
end 

function RevoluteJoint(id, plink_id, clink_id, axis, position, rotmat)
    position = SVector3f(position)
    pose = Transform(position, rotmat)
    common = JointCommonData(id, plink_id, clink_id, pose)
    axis = SVector3f(axis)
    RevoluteJoint(common, axis)
end

#rj = RevoluteJoint(0, 0, 1, [0, 0, 1], [0, 0, 0], [0, 0, 0])

struct FixedJoint<:Joint
    pose::Transform
    axis::SVector3f
end 

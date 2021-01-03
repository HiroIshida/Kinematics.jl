using Rotations: rotation_between, Rotation, RotMatrix, UnitQuaternion, RotXYZ
using StaticArrays
const SVector3f = SVector{3, Float64}

mutable struct Link_
    name::String
    id::Int
    pjoint_id::Int
    cjoint_ids::Vector{Int}
    plink_id::Int
    clink_ids::Vector{Int}
end

struct Link
    name::String
    id::Int
    pjoint_id::Int
    cjoint_ids::Vector{Int}
    plink_id::Int
    clink_ids::Vector{Int}
end
Link(l::Link_) = Link(l.name, l.id, l.pjoint_id, l.cjoint_ids, l.plink_id, l.clink_ids)

mutable struct Transform
    positoin::SVector3f
    rotation::UnitQuaternion
end

function Transform(position::SVector3f, rotmat::AbstractMatrix)
    rotation = UnitQuaternion(rotmat)
    return Transform(position, rotation)
end


abstract type JointType end
for MovableJointType in (:Revolute, :Prismatic)
    # code generation for movable joint type 
    @eval begin
        struct $MovableJointType<:JointType
            axis::SVector3f
            lower_limit::Float64
            upper_limit::Float64
        end

        # define methods related to movable joints

        $MovableJointType(axis, lower_limit, upper_limit) = ($MovableJointType(SVector3f(axis), lower_limit, upper_limit))

        $MovableJointType(axis) = (Revolute(SVector3f(axis), -Inf, Inf))
    end
end
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

mutable struct Mechanism
    links::Vector{Link}
    joints::Vector{Joint}
end

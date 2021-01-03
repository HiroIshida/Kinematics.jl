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
    linkid_map::Dict{String, Int}
    jointid_map::Dict{String, Int}
end
parent_link(m::Mechanism, joint::JointType) = m.links[joint.plink_id]
child_link(m::Mechanism, joint::JointType) = m.links[joint.clink_id]

parent_link(m::Mechanism, link::Link) = m.links[link.plink_id]
child_links(m::Mechanism, link::Link) = m.links[link.clink_ids]
parent_joint(m::Mechanism, link::Link) = m.joints[link.pjoint_id]
child_joints(m::Mechanism, link::Link) = m.joints[link.cjoint_ids]

find_joint(m::Mechanism, joint_name) = m.joints[m.jointid_map[joint_name]]
find_link(m::Mechanism, link_name) = m.links[m.linkid_map[link_name]]
isroot(link::Link) = (link.plink_id==-1)
isleaf(link::Link) = (isempty(link.clink_ids))

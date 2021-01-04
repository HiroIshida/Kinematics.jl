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

function Joint(name, id, plink_id, clink_id, transform, jt::JT) where {JT<:JointType}
    pose = Transform(transform)
    Joint{JT}(name, id, plink_id, clink_id, pose, jt)
end

@inline function joint_transform(joint::Joint{Fixed}, plink_to_hjoint::Transform, angle)
    return plink_to_hjoint # is plink_to_hlink
end

function joint_transform(joint::Joint{Revolute}, plink_to_hjoint::Transform, angle)
    angle==0.0 && return plink_to_hjoint # no transform
    q = UnitQuaternion(cos(0.5*angle), (joint.jt.axis * sin(0.5*angle)...))
    return plink_to_hjoint * Transform(q)
end

function joint_transform(joint::Joint{Prismatic}, plink_to_hjoint::Transform, angle)
    angle==0.0 && return plink_to_hjoint # no transform
    return plink_to_hjoint * Transform(SVector3f(joint.jt.axis * angle))
end

mutable struct Mechanism
    links::Vector{Link}
    joints::Vector{Joint}
    linkid_map::Dict{String, Int}
    jointid_map::Dict{String, Int}
    tf_cache::CacheVector{Transform}
    angles::Vector{Float64}

    # these two will be used in forward kinematics computation
    # to "emulate" recursion in avoiding recursive call
    link_id_stack::PseudoStack{Int64}
    tf_stack::PseudoStack{Transform}
end
function Mechanism(links, joints, linkid_map, jointid_map)
    n_links = length(links)
    tf_cache = CacheVector(n_links, zero(Transform))
    angles = zeros(length(joints))
    link_id_stack = PseudoStack(Int64, n_links)
    tf_stack = PseudoStack(Transform, n_links)
    Mechanism(links, joints, linkid_map, jointid_map, tf_cache, angles, link_id_stack, tf_stack)
end

@inbounds @inline parent_link(m::Mechanism, joint::JointType) = m.links[joint.plink_id]
@inbounds @inline child_link(m::Mechanism, joint::JointType) = m.links[joint.clink_id]

@inbounds @inline parent_link(m::Mechanism, link::Link) = m.links[link.plink_id]
@inbounds @inline child_links(m::Mechanism, link::Link) = m.links[link.clink_ids]
@inbounds @inline parent_joint(m::Mechanism, link::Link) = m.joints[link.pjoint_id]
@inbounds @inline child_joints(m::Mechanism, link::Link) = m.joints[link.cjoint_ids]

@inline find_joint(m::Mechanism, joint_name) = m.joints[m.jointid_map[joint_name]]
@inline find_link(m::Mechanism, link_name) = m.links[m.linkid_map[link_name]]
@inline isroot(link::Link) = (link.plink_id==-1)
@inline isleaf(link::Link) = (isempty(link.clink_ids))

@inbounds @inline joint_angle(m::Mechanism, joint::Joint) = m.angles[joint.id]
@inbounds @inline set_joint_angle(m::Mechanism, joint::Joint, angle) = (m.angles[joint.id] = angle)

# forwarding cache methods
@inline invalidate!(m::Mechanism) = invalidate!(m.tf_cache)
@inline set_cache!(m::Mechanism, link::Link, tf) = set_cache!(m.tf_cache, link.id, tf)
@inline set_cache!(m::Mechanism, link_id::Int64, tf) = set_cache!(m.tf_cache, link_id, tf)
@inline get_cache(m::Mechanism, link::Link) = get_cache(m.tf_cache, link.id)
@inline get_cache(m::Mechanism, link_id::Int64) = get_cache(m.tf_cache, link_id)
@inline iscached(m::Mechanism, link::Link) = iscached(m.tf_cache, link.id)

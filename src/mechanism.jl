abstract type GeometricMetaData end

struct BoxMetaData <: GeometricMetaData
    extents::SVector3f
    origin::Transform
end

struct MeshMetaData <: GeometricMetaData
    file_path::String
    origin::Transform
end

mutable struct Link_
    name::String
    id::Int
    pjoint_id::Int
    cjoint_ids::Vector{Int}
    plink_id::Int
    clink_ids::Vector{Int}
    geometric_meta_data::Union{GeometricMetaData, Nothing}
end

abstract type LinkType end
struct URDF<:LinkType end
struct User<:LinkType end

mutable struct Link{LT<:LinkType}
    link_type::LT
    name::String
    id::Int
    pjoint_id::Int
    cjoint_ids::Vector{Int}
    plink_id::Int
    clink_ids::Vector{Int}
    geometric_meta_data::Union{GeometricMetaData, Nothing}
    data::Dict
end
Link(l::Link_) = Link{URDF}(URDF(), l.name, l.id, l.pjoint_id, l.cjoint_ids, l.plink_id, l.clink_ids, l.geometric_meta_data, Dict())

Link(::Type{LT}, name) where LT<:LinkType = Link(LT(), name)
Link(link_type::LT, name) where LT<:LinkType = Link{LT}(link_type, name, -1, -1, [], -1, [], nothing, Dict())

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

        $MovableJointType(axis) = ($MovableJointType(SVector3f(axis), -Inf, Inf))
        lower_limit(jt::$MovableJointType) = jt.lower_limit
        upper_limit(jt::$MovableJointType) = jt.upper_limit
    end
end
struct Fixed<:JointType end
lower_limit(jt::Fixed) = -Inf
upper_limit(jt::Fixed) = Inf

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
lower_limit(joint::Joint) = lower_limit(joint.jt)
upper_limit(joint::Joint) = upper_limit(joint.jt)

@inline function joint_transform(joint::Joint{Fixed}, angle)
    return joint.pose # is plink_to_hlink
end

function joint_transform(joint::Joint{Revolute}, angle)
    angle==0.0 && return joint.pose # no transform
    q = UnitQuaternion(cos(0.5*angle), (joint.jt.axis * sin(0.5*angle)...))
    return joint.pose * Transform(q)
end

function joint_transform(joint::Joint{Prismatic}, angle)
    angle==0.0 && return joint.pose # no transform
    return joint.pose * Transform(SVector3f(joint.jt.axis * angle))
end

struct FloatingAxis
    origin::SVector3f
    axis::SVector3f
end
function Base.one(::Type{FloatingAxis})
    FloatingAxis([0, 0, 0], [1, 0, 0])
end

function Base.zero(::Type{FloatingAxis})
    FloatingAxis([0, 0, 0], [1, 0, 0])
end

function create_rptable(links::Vector{<:Link}, joints::Vector{<:Joint})
    n_link = length(links)
    n_joint = length(joints)

    table = Vector{BitVector}(undef, n_joint)
    for i in 1:n_joint
        table[i] = falses(n_link)
    end

    # For clarity. Because this is not performance critical part.
    function recursion(joint, clink)
        table[joint.id][clink.id] = true
        for clink_id in clink.clink_ids
            recursion(joint, links[clink_id])
        end
    end

    for joint in joints
        clink = links[joint.clink_id]
        recursion(joint, clink)
    end
    return table
end

abstract type RobotType end
struct Basic <: RobotType
    name::Symbol
end
Basic() = Basic(:basic)

mutable struct Mechanism{RT<:RobotType}
    robot_type::RT
    links::Vector{Link}
    joints::Vector{Joint}
    linkid_map::Dict{String, Int}
    jointid_map::Dict{String, Int}
    tf_cache::CacheVector{Transform}
    axis_cache::CacheVector{FloatingAxis}
    angles::Vector{Float64}
    base_pose::MVector{3, Float64}
    with_base::Bool

    rptable::Vector{BitVector}

    # these two will be used in forward kinematics computation
    # to "emulate" recursion in avoiding recursive call
    link_id_stack::PseudoStack{Int64}
    tf_stack::PseudoStack{Transform}
end
function Mechanism(links, joints, linkid_map, jointid_map, with_base, robot_type::RT) where RT<:RobotType
    n_links = length(links)
    n_joints = length(joints)
    tf_cache = CacheVector(n_links, zero(Transform)) # TODO should be zero -> one
    axis_cache = CacheVector(n_joints, one(FloatingAxis))
    base_pose = MVector{3, Float64}(0.0, 0.0, 0.0)

    rptable = create_rptable(links, Vector{Joint}(joints)) # TODO any better way?

    angles = zeros(length(joints))
    link_id_stack = PseudoStack(Int64, n_links)
    tf_stack = PseudoStack(Transform, n_links)
    Mechanism{RT}(robot_type, links, joints, linkid_map, jointid_map,
        tf_cache, axis_cache,
        angles, base_pose, with_base, rptable, link_id_stack, tf_stack)
end

@inbounds @inline parent_link(m::Mechanism, joint::Joint) = m.links[joint.plink_id]
@inbounds @inline child_link(m::Mechanism, joint::Joint) = m.links[joint.clink_id]

@inbounds @inline parent_link(m::Mechanism, link::Link) = m.links[link.plink_id]
@inbounds @inline child_links(m::Mechanism, link::Link) = m.links[link.clink_ids]
@inbounds @inline parent_joint(m::Mechanism, link::Link) = m.joints[link.pjoint_id]
@inbounds @inline child_joints(m::Mechanism, link::Link) = m.joints[link.cjoint_ids]

@inline find_joint(m::Mechanism, joint_name) = m.joints[m.jointid_map[joint_name]]
@inline find_link(m::Mechanism, link_name) = m.links[m.linkid_map[link_name]]
@inline isroot(link::Link) = (link.plink_id==-1)
@inline isleaf(link::Link) = (isempty(link.clink_ids))
@inline name(m::Mechanism) = m.robot_type.name

@inbounds @inline joint_angle(m::Mechanism, joint::Joint) = m.angles[joint.id]

@inbounds @inline set_joint_angle(m::Mechanism, joint::Joint, angle) = (set_joint_angle(m, joint.id, angle))
@inbounds set_joint_angle(m::Mechanism, joint_id::Int, angle) = (m.angles[joint_id] = angle; invalidate_cache!(m))
@inline set_base_pose(m::Mechanism, vec::AbstractArray) = (m.base_pose = MVector3f(vec); invalidate_cache!(m))

function get_joint_angles!(m::Mechanism, joints::Vector{<:Joint}, angle_vector)
    n_joints = length(joints)
    @debugassert length(angle_vector) == n_joints + (m.with_base ? 3 : 0)
    for i in 1:n_joints
        angle_vector[i] = m.angles[joints[i].id]
    end
    if m.with_base
        for i in 1:3
            angle_vector[n_joints+i] = m.base_pose[i]
        end
    end
end

function get_joint_angles(m::Mechanism, joints::Vector{<:Joint})
    n_dof = length(joints) + (m.with_base ? 3 : 0)
    angle_vector = zeros(n_dof)
    get_joint_angles!(m, joints, angle_vector)
    return angle_vector
end

function set_joint_angles(m::Mechanism, joints::Vector{<:Joint}, angles)
    @debugassert length(joints) + (m.with_base ? 3 : 0) == length(angles) 
    jangle = (m.with_base ? (@view angles[1:end-3]) : angles)
    for (joint, a) in zip(joints, jangle)
        m.angles[joint.id] = a
    end
    m.with_base && (m.base_pose = MVector3f(@view angles[end-2:end]))
    invalidate_cache!(m)
end

function add_new_link(m::Mechanism, new_link::Link, parent::Link, position::AbstractVector)
    pose = Transform(SVector3f(position))
    add_new_link(m, new_link, parent, pose)
end

function add_new_link(m::Mechanism, new_link::Link, parent::Link, pose::Transform)
    hlink_id = length(m.links) + 1
    push!(parent.clink_ids, hlink_id)

    joint_name = new_link.name * "_joint"
    joint_id = length(m.joints) + 1
    plink_id = parent.id
    clink_id = hlink_id

    new_fixed_joint = Joint(joint_name, joint_id, plink_id, clink_id, pose, Fixed())

    new_link.id = hlink_id
    new_link.pjoint_id = joint_id
    new_link.cjoint_ids = []
    new_link.plink_id = parent.id
    new_link.clink_ids = []
    new_link.geometric_meta_data = nothing
    new_link.data = Dict()

    push!(m.links, new_link)
    push!(m.joints, new_fixed_joint)
    m.linkid_map[new_link.name] =  new_link.id
    m.jointid_map[new_fixed_joint.name] = new_fixed_joint.id

    extend!(m.tf_cache)
    extend!(m.axis_cache)
    m.rptable = create_rptable(m.links, m.joints)
    push!(m.angles, 0.0)
    invalidate_cache!(m)
    # TODO maybe tf_stack and link_id_stack should be updated
end
#
# forwarding cache methods
@inline invalidate_cache!(m::Mechanism) = (invalidate!(m.tf_cache); invalidate!(m.axis_cache))
@inline set_cache!(m::Mechanism, link::Link, tf) = set_cache!(m.tf_cache, link.id, tf)
@inline set_cache!(m::Mechanism, link_id::Int64, tf) = set_cache!(m.tf_cache, link_id, tf)
@inline get_cache(m::Mechanism, link::Link) = get_cache(m.tf_cache, link.id)
@inline get_cache(m::Mechanism, link_id::Int64) = get_cache(m.tf_cache, link_id)
@inline iscached(m::Mechanism, link::Link) = iscached(m.tf_cache, link.id)

@inline is_relevant(m::Mechanism, joint::Joint, link::Link) = m.rptable[joint.id][link.id]

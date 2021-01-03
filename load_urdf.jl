using PyCall
@pyimport(skrobot)
robot_model = skrobot.models.PR2()
urdf_model = skrobot.utils.URDF.load(robot_model.urdf_path)

include("joint.jl")

struct Box
    origin
    width
end

struct Mesh
    file_path
end

struct Link
    joint_id
    parent_link_id
    child_link_id
end

function generate_geometry_from_urdfpy(urdfpy_geometry)
    if urdfpy_geometry.mesh
    elseif urdfpy_geometry.box
    end
end

joint_map = Dict()
for i in 1:length(urdf_model.joints)
    joint = urdf_model.joints[i]
    joint_map[joint.name] = i
end

link_map = Dict()
for i in 1:length(urdf_model.links)
    link = urdf_model.links[i]
    link_map[link.name] = i
end

# not construct links
joints = []
for urdf_joint in urdf_model.joints
    id = joint_map[urdf_joint.name]
    plink_id = link_map[urdf_joint.parent]
    clink_id = link_map[urdf_joint.child]
    position = urdf_joint.origin[1:3, 4]
    rotmat = urdf_joint.origin[1:3, 1:3]

    function get_joint_type(joint_type_name)
        if joint_type_name=="revolute"
            lower_limit = urdf_joint.limit.lower
            upper_limit = urdf_joint.limit.upper
            return Revolute(urdf_joint.axis, lower_limit, upper_limit)
        elseif joint_type_name=="prismatic"
            lower_limit = urdf_joint.limit.lower
            upper_limit = urdf_joint.limit.upper
            return Prismatic(urdf_joint.axis, lower_limit, upper_limit)
        else
            return Fixed()
        end
    end
    jt = get_joint_type(urdf_joint.joint_type)
    joint = Joint(urdf_model.name, id, plink_id, clink_id, position, rotmat, jt)
    push!(joints, joint)
end


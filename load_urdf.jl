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
    child_link_ids
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
joint_list = []
for urdf_joint in urdf_model.joints
    id = joint_map[urdf_joint.name]
    parent_link_id = link_map[urdf_joint.parent]
    child_link_id = link_map[urdf_joint.child]
    axis = urdf_joint.axis
    position = urdf_joint.origin[1:3, 4]
    rotmat = urdf_joint.origin[1:3, 1:3]
    if urdf_joint.joint_type=="revolute"
        joint = RevoluteJoint(id, parent_link_id, child_link_id, axis, position, rotmat)
        push!(joint_list, joint)
    #elseif urdf_joint.joint_type=="fixed"
        #joint = FixedJoint(id, parent_link_id, child_link_ids)
    end

end

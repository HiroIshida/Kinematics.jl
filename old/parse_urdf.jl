using LightXML

struct InvalidJonintException <: Exception end

struct Pose
    position
    rotation
end
function Pose(xml_pose::XMLElement)
    dict = attributes_dict(xml_pose)
    (haskey(dict, "xyz") && haskey(dict, "rpy")) || throw(InvalidJonintException)
    xyz = dict["xyz"]
    rpy = dict["rpy"]
    Pose(xyz, rpy)
end

struct Joint
    id
    name
    type
    parent_to_joint_origin_transform::Pose
    parent_link_id
    child_link_id
    # limits
end

function Joint(xml_joint::XMLElement, link_id_table, joint_id_table)
    dict = attributes_dict(xml_joint)
    name = dict["name"]
    id = joint_id_table[name]
    type = dict["type"]

    parent_link_name = attribute(find_element(xml_joint, "parent"), "link")
    child_link_name = attribute(find_element(xml_joint, "child"), "link")
    parent_link_id = link_id_table[parent_link_name]
    child_link_id = link_id_table[child_link_name]
    
    xml_pose = find_element(xml_joint, "origin")
    parent_to_joint_origin_transform = Pose(xml_pose)

    Joint(id, name, type, parent_to_joint_origin_transform,
          parent_link_id, child_link_id)
end


struct Link
    id
    name
    parent_joint
    parent_link_id
    child_link_ids
end

function Link(xml_link::XMLElement, child_link_names, link_id_table, joint_id_table)
    dict = attributes_dict(xml_link)
    name = dict["name"]
    id = link_id_table[name]
    Link(id, name, -1, [])
end


mutable struct RobotModel
    link_id_table
    joint_id_table
    links
    joints
end
function RobotModel(urdf_path)
    xdoc = parse_file(urdf_path)
    xroot = root(xdoc)
    xml_links = get_elements_by_tagname(xroot, "link")
    xml_joints = get_elements_by_tagname(xroot, "joint")

    link_id_table = Dict()
    counter = 1
    for xml_link in xml_links
        link_name = attributes_dict(xml_link)["name"]
        link_id_table[link_name] = counter
        counter += 1
    end

    joint_id_table = Dict()
    counter = 1
    for xml_joint in xml_joints
        joint_name = attributes_dict(xml_joint)["name"]
        joint_id_table[joint_name] = counter
        counter += 1
    end

    joints = []
    for xml_joint in xml_joints
        try
            joint = Joint(xml_joint, link_id_table, joint_id_table)
            push!(joints, joint)
        catch e
            println("invalid joint is skipped")
        end
    end

    links = []
    for xml_link in xml_links
        link = Link(xml_link, link_id_table, joint_id_table)
        push!(links, link)
    end

    
end
robot = RobotModel("fetch.urdf")


"""
struct Collision
    xyz
    rpy
    geometry
    function from_xml(xml_element)
    end
end

struct Inertial
    mass
    origin
end

struct Link
    name
    collision::Collision

    function from_xml(xml_element)
        name = name(xml_element)
    end
end

link = link_xml_elements[70]
attribute_dict(link)

"""

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
    parent_link_name
    child_link_name
    parent_link_id
    child_link_id
    # limits
end

struct Link
    id
    name
end
function Link(xml_link::XMLElement)
    id = -1
    dict = attributes_dict(xml_link)
    name = dict["name"]
    Link(id, name)
end

function Joint(xml_joint::XMLElement)
    dict = attributes_dict(xml_joint)
    id = -1
    name = dict["name"]
    type = dict["type"]
    println(type)
    parent_link_name = attribute(find_element(xml_joint, "parent"), "link")
    child_link_name = attribute(find_element(xml_joint, "child"), "link")
    xml_pose = find_element(xml_joint, "origin")
    parent_to_joint_origin_transform = Pose(xml_pose)
    parent_link_id = -1
    child_link_id = -1
    Joint(id, name, type, parent_to_joint_origin_transform,
          parent_link_name, child_link_name, parent_link_id, child_link_id)
end


mutable struct RobotModel
    link_id_table
    joint_id_table
    link_arr
    joint_arr
end
function RobotModel(urdf_path)
    xdoc = parse_file(urdf_path)
    xroot = root(xdoc)
    xml_links = get_elements_by_tagname(xroot, "link")
    xml_joints = get_elements_by_tagname(xroot, "joint")

    joint_arr = []
    for xml_joint in xml_joints
        try
            joint = Joint(xml_joint)
            push!(joint_arr, joint)
        catch e
            println("invalid joint is skipped")
        end
    end

    link_arr = []
    for xml_link in xml_links
        link = Link(xml_link)
        push!(link_arr, link)
    end

    link_id_table = Dict()
    for id in 1:length(joint_arr)
        name = joint_arr[id].name
        link_id_table[name] = id
    end

    joint_id_table = Dict()
    for id in 1:length(joint_arr)
        name = joint_arr[id].name
        joint_id_table[name] = id
    end
    return RobotModel(link_id_table, joint_id_table, link_arr, joint_arr)
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

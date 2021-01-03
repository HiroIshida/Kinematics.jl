function parse_urdf(urdf_path)
    urdf_model = __skrobot__.utils.URDF.load(urdf_path)
    jointid_map = Dict()
    for i in 1:length(urdf_model.joints)
        joint = urdf_model.joints[i]
        jointid_map[joint.name] = i
    end

    linkid_map = Dict()
    for i in 1:length(urdf_model.links)
        link = urdf_model.links[i]
        linkid_map[link.name] = i
    end

    links_tmp = []
    for urdf_link in urdf_model.links
        id = linkid_map[urdf_link.name]
        push!(links_tmp, Link_(urdf_link.name, id, -1, [], -1, []))
    end

    joints = []
    for urdf_joint in urdf_model.joints
        id = jointid_map[urdf_joint.name]
        plink_id = linkid_map[urdf_joint.parent]
        clink_id = linkid_map[urdf_joint.child]
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
        joint = Joint(urdf_joint.name, id, plink_id, clink_id, position, rotmat, jt)
        push!(joints, joint)
        push!(joints, joint)

        plink = links_tmp[joint.plink_id]
        push!(plink.cjoint_ids, joint.id)
        push!(plink.clink_ids, joint.clink_id)

        clink = links_tmp[joint.clink_id]
        clink.pjoint_id = joint.id
        clink.plink_id = joint.plink_id
    end

    links = map(Link, links_tmp)
    mech = Mechanism(links, joints, linkid_map, jointid_map)
end

"""
robot_model = skrobot.models.PR2()
mech = parse_urdf(robot_model.urdf_path)
"""

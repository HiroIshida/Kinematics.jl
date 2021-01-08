function geo_mata_from_urdf_link(geom_urdf)
    mesh = geom_urdf.collision_mesh
    if mesh != nothing
        is_loaded_from_file = haskey(mesh.metadata, "file_path")
        if is_loaded_from_file
            file_path = mesh.metadata["file_path"]
            return MeshMetaData(file_path, Transform(mesh.metadata["origin"]))
        else # primitives
            primitive_type = mesh.metadata["shape"]
            if primitive_type=="box"
                extents = mesh.metadata["extents"]
                return BoxMetaData(extents, Transform(mesh.metadata["origin"]))
            else
                println("primitive type other than box is not supported yet")
            end
        end
    end
end

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
        geo_meta = geo_mata_from_urdf_link(urdf_link)
        push!(links_tmp, Link_(urdf_link.name, id, -1, [], -1, [], geo_meta))
    end

    joints = []
    for urdf_joint in urdf_model.joints
        id = jointid_map[urdf_joint.name]
        plink_id = linkid_map[urdf_joint.parent]
        clink_id = linkid_map[urdf_joint.child]
        transform = urdf_joint.origin

        function get_joint_type(joint_type_name)
            if joint_type_name=="revolute"
                lower_limit = urdf_joint.limit.lower
                upper_limit = urdf_joint.limit.upper
                return Revolute(urdf_joint.axis, lower_limit, upper_limit)
            elseif joint_type_name=="continuous"
                return Revolute(urdf_joint.axis, -Inf, Inf)
            elseif joint_type_name=="prismatic"
                lower_limit = urdf_joint.limit.lower
                upper_limit = urdf_joint.limit.upper
                return Prismatic(urdf_joint.axis, lower_limit, upper_limit)
            elseif joint_type_name=="fixed"
                return Fixed()
            else
                throw(Exception)
            end
        end
        jt = get_joint_type(urdf_joint.joint_type)
        joint = Joint(urdf_joint.name, id, plink_id, clink_id, transform, jt)
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

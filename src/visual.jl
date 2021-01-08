to_affine_map(tform::Transform) = AffineMap(rotation(tform), translation(tform))

function add_mechanism(vis::Visualizer, mech::Mechanism)
    for link in mech.links
        mesh = link.urdf_link.collision_mesh
        tf_world_to_link = get_transform(mech, link)
        if mesh!=nothing
            tf_link_to_geom = Transform(mesh.metadata["origin"])
            tf_world_to_geom = tf_world_to_link * tf_link_to_geom

            is_loaded_from_file = haskey(mesh.metadata, "file_path")
            if is_loaded_from_file
                file_path = mesh.metadata["file_path"]
                geometry = MeshFileGeometry(file_path)
                setobject!(vis[link.name], geometry)
                settransform!(vis[link.name], to_affine_map(tf_world_to_geom))
            else # primitives
                primitive_type = mesh.metadata["shape"]
                if primitive_type=="box"
                    extents = mesh.metadata["extents"]
                    geometry = Rect(Vec(0., 0, 0), Vec(extents...))
                    setobject!(vis[link.name], geometry)
                    settransform!(vis[link.name], to_affine_map(tf_world_to_geom))
                else
                    println("primitive type other than box is not supported yet")
                end
            end
        end
    end
    nothing
end

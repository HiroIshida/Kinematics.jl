to_affine_map(tform::Transform) = AffineMap(rotation(tform), translation(tform))

create_vis_object(md::BoxMetaData) = (Rect(Vec(0, 0, 0.), Vec(md.extents...)))
create_vis_object(md::MeshMetaData) = (MeshFileGeometry(md.file_path))

function update(vis::Visualizer, mech::Mechanism)
    for link in mech.links
        update(vis, mech, link)
    end
    nothing
end

function update(vis::Visualizer, mech::Mechanism, link::Link)
    isnothing(link.geometric_meta_data) && return 

    tf_world_to_link = get_transform(mech, link)
    tf_link_to_geom = link.geometric_meta_data.origin
    tf_world_to_geom = tf_world_to_link * tf_link_to_geom
    settransform!(vis[link.name], to_affine_map(tf_world_to_geom))
end

function add_mechanism(vis::Visualizer, mech::Mechanism)
    for link in mech.links
        add_link(vis, link)
        update(vis, mech, link)
    end
    nothing
end

function add_link(vis::Visualizer, link::Link)
    isnothing(link.geometric_meta_data) && return 
    vis_obj = create_vis_object(link.geometric_meta_data)
    setobject!(vis[link.name], vis_obj)
end


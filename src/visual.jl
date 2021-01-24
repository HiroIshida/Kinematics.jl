to_affine_map(tform::Transform) = AffineMap(rotation(tform), translation(tform))

create_vis_object(md::BoxMetaData) = (Rect(Vec(0, 0, 0.), Vec(md.extents...)))
create_vis_object(md::MeshMetaData) = (MeshFileGeometry(md.file_path))

create_vis_sphere(radius) = (HyperSphere(Point(0, 0, 0.), radius))

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


function rgb_material(rgb::AbstractArray)
    material = MeshCat.defaultmaterial()
    material.color = RGB(rgb...)
    return material
end
red_material() = rgb_material([1, 0, 0])
blue_material() = rgb_material([0, 1, 0])
green_material() = rgb_material([0, 0, 1])

function add_frame(vis::Visualizer, tf_world_to_here::Transform; length=0.2)
    avis1 = ArrowVisualizer(vis[:xaxis])
    avis2 = ArrowVisualizer(vis[:yaxis])
    avis3 = ArrowVisualizer(vis[:zaxis])

    setobject!(avis1, red_material())
    setobject!(avis2, blue_material())
    setobject!(avis3, green_material())
    origin = translation(tf_world_to_here)
    rotmat = rotation(tf_world_to_here)

    e1 = rotmat[:, 1] * length
    e2 = rotmat[:, 2] * length
    e3 = rotmat[:, 3] * length
    settransform!(avis1, Point(origin...), Vec(e1...))
    settransform!(avis2, Point(origin...), Vec(e2...))
    settransform!(avis3, Point(origin...), Vec(e3...))
end

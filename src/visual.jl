using MeshCat
import Lazy 

to_affine_map(tform::Transform) = AffineMap(rotation(tform), translation(tform))

create_vis_object(md::BoxMetaData) = (Rect(Vec((-0.5*md.extents)...), Vec(1.0*md.extents...)))
create_vis_object(md::MeshMetaData) = (MeshFileGeometry(md.file_path))

create_vis_sphere(radius) = (HyperSphere(Point(0, 0, 0.), radius))

function update(vis::AbstractVisualizer, mech::Mechanism)
    for link in mech.links
        update(vis[name(mech)], mech, link)
    end
    nothing
end

function update(vis::AbstractVisualizer, mech::Mechanism, link::Link)
    isnothing(link.geometric_meta_data) && return 
    update_common(vis, mech, link, link.geometric_meta_data.origin)
end

function update_common(vis::AbstractVisualizer, mech::Mechanism, link::Link, origin::Transform)
    tf_world_to_link = get_transform(mech, link)
    tf_link_to_geom = origin
    tf_world_to_geom = tf_world_to_link * tf_link_to_geom
    settransform!(vis[link.name], to_affine_map(tf_world_to_geom))
end

function add_sdf(vis::AbstractVisualizer, boxsdf::BoxSDF)
    MeshCat.setobject!(vis[:sdf], Rect(Vec(-0.5*boxsdf.width), Vec(boxsdf.width)))
    settransform!(vis[:sdf], to_affine_map(boxsdf.pose))
end

function add_mechanism(vis::AbstractVisualizer, mech::Mechanism)
    for link in mech.links
        add_link(vis[name(mech)], link)
        update(vis, mech, link)
    end
    nothing
end

function add_link(vis::AbstractVisualizer, link::Link)
    isnothing(link.geometric_meta_data) && return 
    vis_obj = create_vis_object(link.geometric_meta_data)
    MeshCat.setobject!(vis[link.name], vis_obj)
end


function rgb_material(rgb::AbstractArray)
    material = MeshCat.defaultmaterial()
    material.color = RGB(rgb...)
    return material
end
red_material() = rgb_material([1, 0, 0])
green_material() = rgb_material([0, 1, 0])
blue_material() = rgb_material([0, 0., 1.])
yellow_material() = rgb_material([1., 1., 0.])

function add_frame(vis::AbstractVisualizer, tf_world_to_here::Transform; length=0.2)
    avis1 = ArrowVisualizer(vis[:xaxis])
    avis2 = ArrowVisualizer(vis[:yaxis])
    avis3 = ArrowVisualizer(vis[:zaxis])

    MeshCat.setobject!(avis1, red_material())
    MeshCat.setobject!(avis2, blue_material())
    MeshCat.setobject!(avis3, green_material())
    origin = translation(tf_world_to_here)
    rotmat = rotation(tf_world_to_here)

    e1 = rotmat[:, 1] * length
    e2 = rotmat[:, 2] * length
    e3 = rotmat[:, 3] * length
    settransform!(avis1, Point(origin...), Vec(e1...))
    settransform!(avis2, Point(origin...), Vec(e2...))
    settransform!(avis3, Point(origin...), Vec(e3...))
end

struct MechanismVisualizer <: MeshCat.AbstractVisualizer
    vis::MeshCat.Visualizer
    mechs::Vector{Mechanism}
end
MechanismVisualizer() = MechanismVisualizer(Visualizer(), Vector{Mechanism}(undef, 0))
Lazy.@forward MechanismVisualizer.vis Base.getindex, MeshCat.setobject!, MeshCat.open

function add_mechanism(mvis::MechanismVisualizer, mech::Mechanism)
    add_mechanism(mvis.vis, mech)
    push!(mvis.mechs, mech)
end

function update(mvis::MechanismVisualizer)
    for mech in mvis.mechs
        update(mvis, mech)
    end
end

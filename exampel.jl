using Revise
using StaticArrays
using Kinematics
import JSON
using Test 

# benchmarking
urdf_path = Kinematics.__skrobot__.data.fetch_urdfpath()
mech = parse_urdf(urdf_path)

link_names = ["l_gripper_finger_link", "r_gripper_finger_link", "wrist_flex_link", "wrist_roll_link", "shoulder_lift_link", "upperarm_roll_link"];
links = [find_link(mech, name) for name in link_names]

function bench(mech, links)
    for i in 1:1000000
        invalidate_cache!(mech)
        for link in links
            tf = get_transform(mech, link)
        end
    end
end
bench(mech, links) # dryrun

using BenchmarkTools
@time bench(mech, links)

using MeshCat
using GeometryBasics
using CoordinateTransformations
using LinearAlgebra

to_affine_map(tform::Transform) = AffineMap(rotation(tform), translation(tform))


function add_link_to_vis(link, vis)
    mesh = link.urdf_link.collision_mesh
    tf_world_to_link = get_transform(mech, link)
    if mesh!=nothing
        tf_link_to_geom = Transform(mesh.metadata["origin"])
        tf_world_to_geom = tf_world_to_link * tf_link_to_geom

        is_loaded_from_file = haskey(mesh.metadata, "file_path")
        if is_loaded_from_file
            file_path = mesh.metadata["file_path"]
            println(file_path)
            geometry = MeshFileGeometry(file_path)
            setobject!(vis[link.name], geometry)
            settransform!(vis[link.name], to_affine_map(tf_world_to_geom))
        else # primitives
            primitive_type = mesh.metadata["shape"]
            if primitive_type=="box"
                extents = mesh.metadata["extents"]
                geometry = Rect(Vec(0., 0, 0), Vec(extents...))
                setobject!(vis[link.name], geometry)
                println(translation(tf_world_to_geom))
                settransform!(vis[link.name], to_affine_map(tf_world_to_geom))
            else
                println("primitive type other than box is not supported yet")
            end
        end
    end
end


vis = Visualizer()

joint = find_joint(mech, "shoulder_lift_joint")
set_joint_angle(mech, joint, -1.0)
invalidate_cache!(mech)
for link in mech.links
    add_link_to_vis(link, vis)
end
open(vis)

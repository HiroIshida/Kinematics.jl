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
        invalidate!(mech)
        for link in links
            tf = get_transform(mech, link)
        end
    end
end
bench(mech, links) # dryrun

using BenchmarkTools
@time bench(mech, links)
links = mech.links
ulink = links[1].urdf_link
mesh = ulink.collision_mesh
path = mesh.metadata["file_path"]

using MeshCat
using CoordinateTransformations


function add_link_to_vis(link, vis)
    mesh = link.urdf_link.collision_mesh
    if mesh!=nothing
        is_loaded_from_file = haskey(mesh.metadata, "file_path")
        if is_loaded_from_file
            file_path = mesh.metadata["file_path"]
            println(file_path)
            geometry = MeshFileGeometry(file_path)
            setobject!(vis[link.name], geometry)
            
            tf = get_transform(mech, link)
            to_affine_map(tform::Transform) = AffineMap(rotation(tform), translation(tform))
            settransform!(vis[link.name], to_affine_map(tf))
        else # primitives
            primitive_type = mesh.metadata["shape"]
            if primitive_type=="box"
                extents = mesh.metadata["extents"]
            else
                println("primitive type other than box is not supported yet")
            end
            """ python code
            elif shape == 'cylinder':
                height = mesh.metadata['height']
                radius = mesh.metadata['radius']
                sdf = CylinderSDF([0, 0, 0], radius=radius, height=height)
            elif shape == 'sphere':
                radius = mesh.metadata['radius']
                sdf = SphereSDF([0, 0, 0], radius)
            """
        end
    end
end

vis = Visualizer()
for link in links
    add_link_to_vis(link, vis)
        """
        origin = Transform(mesh.metadata["origin"])
        tf = get_transform(mech, link)

        setobject!(vis, mesh_file_objects[1])
        """
end
#open(vis)

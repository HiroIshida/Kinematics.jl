using UUIDs
using Revise
using MeshCat
using Kinematics
urdf_path = "./data/complecated_fridge.urdf"
fridge = parse_urdf(urdf_path, with_base=true)
sdf = UnionSDF(fridge)

robot = load_pr2(with_base=true)
joints = rarm_joints(robot)
collision_links = rarm_collision_links(robot)
sscc = SweptSphereCollisionChecker(robot)
for link in collision_links
    add_coll_links(sscc, link)
end

vis = Visualizer()
#add_mechanism(vis, fridge)
add_mechanism(vis, robot)

joint = find_joint(fridge, "door_joint")
reset_manip_pose(robot)
set_joint_angles(fridge, [joint], [0.7, 1.5, 0.0, 0.0])
#set_joint_angles(fridge, [joint], [0.0, 1.5, 0.0, 0.0])

open(vis)
update(vis, robot)
update(vis, fridge)

center = [1.5, 0., 0.]
b_min = center + [-1., -1., 0]
b_max = center + [+1., +1., 2.]
w = b_max - b_min
verts = [Kinematics.SVector3f(b_min + w .* rand(3)) for _ in 1:1000000]
vals = [sdf(v) for v in verts]
verts_extracted = verts[vals .< 0.0]
println("computed")

colors = [Kinematics.RGB(1., 0., 0.0) for _ in verts_extracted]
setobject!(vis, PointCloud(verts_extracted, colors))

tf = get_transform(fridge, sdf.attach_style.link)
translation(tf)

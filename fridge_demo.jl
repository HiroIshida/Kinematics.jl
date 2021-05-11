using UUIDs
using Revise
using MeshCat
using StaticArrays
using Kinematics

const SVector3f = SVector{3, Float64}

urdf_path = "./data/complecated_fridge.urdf"
fridge = parse_urdf(urdf_path, with_base=true)
sdf = UnionSDF(fridge)

robot = load_pr2(with_base=true)
joints = vcat(rarm_joints(robot), larm_joints(robot))
collision_links = vcat(larm_collision_links(robot), rarm_collision_links(robot))
sscc = SweptSphereCollisionChecker(robot)
for link in collision_links
    add_coll_links(sscc, link)
end

vis = Visualizer()
add_mechanism(vis, fridge)
add_mechanism(vis, robot)

joint = find_joint(fridge, "door_joint")
reset_manip_pose(robot)
set_joint_angles(fridge, [joint], [2.0, 1.2, 0.0, 0.0])

tf_fridge = get_transform(fridge, find_link(fridge, "base_link"))
tf_target = Transform(SVector3f([0.0, 0.0, 1.2])) * tf_fridge
add_frame(vis[:finger], tf_target)

q_start = get_joint_angles(robot, joints)
link = find_link(robot, "l_gripper_tool_frame")
inverse_kinematics!(robot, link, joints, tf_target; with_rot=true) # pre solving
q_goal = inverse_kinematics!(robot, link, joints, tf_target; with_rot=true, sscc=sscc, sdf=sdf)

n_wp = 10
margin = 0.03
@time q_seq, status = plan_trajectory(sscc, joints, sdf, q_start, q_goal, n_wp, ftol_abs=1.0e-4, solver=:NLOPT, partial_consts=[], margin=margin)

open(vis)
update(vis, robot)
update(vis, fridge)

for i in 1:n_wp
    sleep(1.0)
    set_joint_angles(robot, joints, q_seq[:, i])
    update(vis, robot)
end

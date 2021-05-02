using Revise
using Kinematics
using StaticArrays
using BenchmarkTools
using Profile
using MeshCat

with_base = true

robot = load_pr2(with_base=with_base)
joints = rarm_joints(robot)
collision_links = rarm_collision_links(robot)
sscc = SweptSphereCollisionChecker(robot)
for link in collision_links
    add_coll_links(sscc, link)
end

q_start = [0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63]
q_goal = [-0.78,  0.055, -1.34, -0.594, -0.494, -0.20, 1.88]
if with_base
    append!(q_start, zeros(3))
    append!(q_goal, zeros(3))
end

box_pose = Transform(Kinematics.SVector3f(0.9, -0.2, 0.9))
box_width = Kinematics.SVector3f(0.7, 0.5, 0.6)
box_sdf = BoxSDF(box_pose, box_width)

n_wp = 10
margin = 0.03
@time q_seq, status = plan_trajectory(sscc, joints, box_sdf, q_start, q_goal, n_wp, ftol_abs=1.0e-4, solver=:NLOPT, partial_consts=[], margin=margin)

vis = Visualizer()
#add_frame(vis[:target], target_pose)
#add_frame(vis[:waypoint], waypoint)
add_sdf(vis, box_sdf)
add_mechanism(vis, robot)
open(vis)
for i in 1:n_wp
    sleep(0.6)
    set_joint_angles(robot, joints, q_seq[:, i])
    update(vis, robot)
end

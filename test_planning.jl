using Revise
using Kinematics
using StaticArrays
using BenchmarkTools
using Profile
using MeshCat

with_base = false

urdf_path = Kinematics.__skrobot__.data.fetch_urdfpath()
mech = parse_urdf(urdf_path, with_base=with_base)
joint_names = [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint"];
joints = [find_joint(mech, name) for name in joint_names]

sscc = SweptSphereCollisionChecker(mech)
add_coll_links(sscc, find_link(mech, "wrist_flex_link"))
add_coll_links(sscc, find_link(mech, "torso_lift_link"))
add_coll_links(sscc, find_link(mech, "upperarm_roll_link"))
add_coll_links(sscc, find_link(mech, "elbow_flex_link"))

pose = Transform(Kinematics.SVector3f(0.4, -0.25, 0.7))
width = Kinematics.SVector3f(0.05, 0.05, 0.5)
boxsdf = BoxSDF(pose, width)

n_wp = 10
q_start = get_joint_angles(mech, joints)

link = find_link(mech, "gripper_link")
target_pose = Transform(Kinematics.SVector3f(0.3, -0.4, 1.2))
q_goal = inverse_kinematics!(mech, link, joints, target_pose; with_rot=false)
set_joint_angles(mech, joints, q_goal)

solver = :NLOPT
xi_init = Kinematics.create_straight_trajectory(q_start, q_goal, n_wp)
@btime q_seq, status = plan_trajectory(sscc, joints, boxsdf, q_start, q_goal, n_wp, ftol_abs=1e-3, solver=solver)

vis = Visualizer()
add_sdf(vis, boxsdf)
add_mechanism(vis, mech)
open(vis)
for i in 1:n_wp
    sleep(0.6)
    set_joint_angles(mech, joints, q_seq[:, i])
    update(vis, mech)
end

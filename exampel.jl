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


add_new_link(mech, links[1], "mylink", [0.2, 0, 0])


function bench(mech, links)
    for i in 1:100000
        invalidate_cache!(mech)
        for link in links
            tf = get_transform(mech, link)
        end
    end
end
bench(mech, links) # dryrun

using BenchmarkTools
@time bench(mech, links)


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
angles = [0.0 for _ in 1:8]

function bench_jacobian(mech, links, joints)
    J = zeros(3, length(joints))
    for i in 1:100000
        set_joint_angles(mech, joints, angles)
        for link in links
            get_jacobian!(mech, link, joints, false, J)
        end
    end
end
@time bench_jacobian(mech, links, joints)

const SVector3f = SVector{3, Float64}
angles_solved = point_inverse_kinematics(mech, find_link(mech, "l_gripper_finger_link"), joints, SVector3f(0.6, 0.3, 1.0))
set_joint_angles(mech, joints, angles_solved)

link_attach = find_link(mech, "torso_lift_link")
ssm = SweptSphereManager()
add_collision_link(ssm, mech, link_attach)


using MeshCat
using GeometryBasics
using CoordinateTransformations

vis = Visualizer()
add_mechanism(vis, mech)
add_collision_spheres(vis, ssm, mech)

tf_roll = get_transform(mech, find_link(mech, "upperarm_roll_link"))
tf_elbow = get_transform(mech, find_link(mech, "l_gripper_finger_link"))
tf_mylink = get_transform(mech, find_link(mech, "mylink"))
add_frame(vis[:finger], tf_elbow)
add_frame(vis[:roll], tf_roll)
add_frame(vis[:mylink], tf_mylink)
open(vis)

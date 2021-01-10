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
        [set_joint_angle(mech, j, a) for (j, a) in zip(joints, angles)]
        for link in links
            get_jacobian!(mech, link, joints, false, J)
        end
    end
end
@time bench_jacobian(mech, links, joints)

using MeshCat
vis = Visualizer()
add_mechanism(vis, mech)
set_joint_angle(mech, find_joint(mech, "upperarm_roll_joint"), 0.4)
invalidate_cache!(mech)
update(vis, mech)

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

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
               "upperarm_roll_joint", "elbow_flex_joint",
               "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
joints = [find_joint(mech, name) for name in joint_names]

J = get_jacobian(mech, find_link(mech, "l_gripper_finger_link"), joints, true)
display(J)


using MeshCat
vis = Visualizer()
add_mechanism(vis, mech)
set_joint_angle(mech, find_joint(mech, "upperarm_roll_joint"), 0.4)
invalidate_cache!(mech)
update(vis, mech)

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
@btime bench(mech, links)

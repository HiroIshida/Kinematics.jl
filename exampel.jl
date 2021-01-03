using Kinematics
import JSON
using Test 

f = open("./data/ground_truth.json", "r")
gtruth_data = JSON.parse(read(f, String))
close(f)

urdf_path = Kinematics.__skrobot__.data.pr2_urdfpath()

mech = parse_urdf(urdf_path)
joints = [find_joint(mech, name) for name in gtruth_data["joint_names"]]
links = [find_link(mech, name) for name in gtruth_data["link_names"]]
angles = gtruth_data["angle_vector"]
poses_gtruth = gtruth_data["pose_list"]

for (joint, angle) in zip(joints, angles)
    set_joint_angle(mech, joint, angle)
end

invalidate!(mech)
for (link, pose_gtruth) in zip(links, poses_gtruth)
    println(link.name)
    tf = get_transform(mech, link)
    @test tf.translation ≈ pose_gtruth[1:3]
end

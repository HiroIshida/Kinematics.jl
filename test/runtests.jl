using Kinematics
using Test 
Kinematics.debugging() = true

# testing cache:
cache = CacheVector(3, 1.0)
@assert typeof(cache) == CacheVector{Float64}
set_cache!(cache, 2, 2.0)
@test get_cache(cache, 2)==2.0
@test !iscached(cache, 1)
@test iscached(cache, 2)
@test_throws AssertionError get_cache(cache, 1)
@test_throws AssertionError set_cache!(cache, 2, 2.0)
Kinematics.invalidate!(cache)
@test_throws AssertionError get_cache(cache, 2)


# testing stack
n_max = 5
ps = PseudoStack(Int64, n_max)
for i in 1:n_max
    push!(ps, i)
end
@test !isempty(ps)
@test pop!(ps) == 5
@test pop!(ps) == 4
@test pop!(ps) == 3
@test pop!(ps) == 2
@test pop!(ps) == 1
@test isempty(ps)

# urdf load test
urdf_path = Kinematics.__skrobot__.data.fetch_urdfpath()
mech = parse_urdf(urdf_path)
base_link = find_link(mech, "base_link")
clink_name_set = Set([l.name for l in child_links(mech, base_link)])
clink_name_set_desired = Set(["r_wheel_link", "l_wheel_link", "torso_lift_link", "estop_link", "laser_link", "torso_fixed_link"])
@test clink_name_set == clink_name_set_desired
@test isroot(base_link)
@test base_link.pjoint_id == -1
leaf_link_names = ["r_wheel_link", "l_wheel_link", "r_gripper_finger_link",
                   "l_gripper_finger_link", "bellows_link2", "estop_link",
                   "laser_link", "torso_fixed_link", "head_camera_rgb_optical_frame",
                   "head_camera_depth_optical_frame"]

shoulder_link = find_link(mech, "shoulder_pan_link")
@test parent_joint(mech, shoulder_link).name == "shoulder_pan_joint"
@test parent_link(mech, shoulder_link).name == "torso_lift_link"
@test length(child_links(mech, shoulder_link)) == 1
@test length(child_joints(mech, shoulder_link)) == 1
@test child_links(mech, shoulder_link)[1].name == "shoulder_lift_link"
@test child_joints(mech, shoulder_link)[1].name == "shoulder_lift_joint"


for name in leaf_link_names
    link = find_link(mech, name)
    @test isleaf(link)
    @test isempty(link.cjoint_ids)
end

urdf_path = Kinematics.__skrobot__.data.pr2_urdfpath()
pr2 = parse_urdf(urdf_path)
froll_link = find_link(pr2, "r_forearm_roll_link")
@test length(child_links(pr2, froll_link)) == 2
@test length(child_joints(pr2, froll_link)) == 2
@test child_links(pr2, froll_link)[1].name == "r_forearm_link"
@test child_links(pr2, froll_link)[2].name == "r_forearm_cam_frame"

# rptable test PR2
urdf_path = Kinematics.__skrobot__.data.pr2_urdfpath()
mech = parse_urdf(urdf_path)

torso_joint = find_joint(mech, "torso_lift_joint")
wrist_joint = find_joint(mech, "r_wrist_flex_joint")
head_link = find_link(mech, "head_pan_link")
caster_link = find_link(mech, "fl_caster_rotation_link")

@test is_relevant(mech, torso_joint, head_link)
@test is_relevant(mech, torso_joint, find_link(mech, "torso_lift_link"))
@test !is_relevant(mech, torso_joint, caster_link)
@test !is_relevant(mech, wrist_joint, head_link)
@test !is_relevant(mech, wrist_joint, caster_link)

# rptable test fetch
urdf_path = Kinematics.__skrobot__.data.fetch_urdfpath()
mech = parse_urdf(urdf_path)
shoulder_joint = find_joint(mech, "shoulder_pan_joint")
wrist_link = find_link(mech, "wrist_roll_link")
base_link = find_link(mech, "base_link")
@test is_relevant(mech, find_joint(mech, "torso_lift_joint"), find_link(mech, "torso_lift_link"))
@test is_relevant(mech, shoulder_joint, wrist_link)
@test !is_relevant(mech, shoulder_joint, base_link)

# kinematics test (testing get_transform)
import JSON
f = open("../data/ground_truth.json", "r")
gtruth_data = JSON.parse(read(f, String))
close(f)

urdf_path = Kinematics.__skrobot__.data.pr2_urdfpath()

mech = parse_urdf(urdf_path)
joints = [find_joint(mech, name) for name in gtruth_data["joint_names"]]
links = [find_link(mech, name) for name in gtruth_data["link_names"]]
angles = gtruth_data["angle_vector"]
poses_gtruth = gtruth_data["pose_list"]

set_joint_angles(mech, joints, angles)
for i in 1:2
    # check two times to check that cache is propery stored
    for (link, pose_gtruth) in zip(links, poses_gtruth)
        println("testing tf ... ")
        println(link.name)
        tf = get_transform(mech, link)
        @test translation(tf) ≈ pose_gtruth[1:3]
    end
end

# kinematics test (testing get_jacobian)
# TODO test quaternion jacobian
eps = 1e-7
for link in mech.links
    set_joint_angles(mech, joints, angles)
    J_analytical = get_jacobian(mech, link, joints, false)
    J_numerical = zeros(3, length(joints))
    pose0 = get_transform(mech, link)
    for i in 1:length(joints)
        angles_ = copy(angles)
        angles_[i] += eps
        set_joint_angles(mech, joints, angles_)
        pose1 = get_transform(mech, link)
        J_numerical[:, i] = (translation(pose1) - translation(pose0))/eps
    end
    println("testing..." * link.name)
    @test isapprox(J_numerical, J_analytical, atol=1e-5) 
    println("[PASS] jacobian of " * link.name)
end


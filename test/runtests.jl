using Kinematics
using Test 
Kinematics.debugging() = true

# testing cache:
cache = CacheVector(3, 1.0)
@assert typeof(cache) == CacheVector{Float64}
set_cache!(cache, 2, 2.0)
@test get_cache(cache, 2)==2.0
@test_throws AssertionError get_cache(cache, 1)
@test_throws AssertionError set_cache!(cache, 2, 2.0)
invalidate!(cache)
@test_throws AssertionError get_cache(cache, 2)


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


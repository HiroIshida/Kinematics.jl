struct PR2 <: RobotType
    name::Symbol
end
PR2() = PR2(:PR2)

function load_pr2(;with_base=false)
    urdf_path = Kinematics.__skrobot__.data.pr2_urdfpath()
    parse_urdf(urdf_path, with_base=with_base, robot_type=PR2())
end

torso_joint(mech::Mechanism{PR2}) = (find_joint(mech, "torso_lift_join"))

function rarm_joints(mech::Mechanism{PR2}; with_torso=false)
    rarm_joint_names = ["r_shoulder_pan_joint", 
                       "r_shoulder_lift_joint",
                       "r_upper_arm_roll_joint",
                       "r_elbow_flex_joint",
                       "r_forearm_roll_joint",
                       "r_wrist_flex_joint",
                       "r_wrist_roll_joint"]
    joints = [find_joint(mech, name) for name in rarm_joint_names]
    with_torso && append!(joints, torso_joint(mech))
    return joints
end

function larm_joints(mech::Mechanism{PR2}; with_torso=false)
    larm_joint_names = ["l_shoulder_pan_joint", 
                       "l_shoulder_lift_joint",
                       "l_upper_arm_roll_joint",
                       "l_elbow_flex_joint",
                       "l_forearm_roll_joint",
                       "l_wrist_flex_joint",
                       "l_wrist_roll_joint"]
    joints = [find_joint(mech, name) for name in larm_joint_names]
    with_torso && append!(joints, torso_joint(mech))
    return joints
end

function rarm_collision_links(mech::Mechanism{PR2})
    rarm_coll_link_names = ["r_upper_arm_link",
                            "r_forearm_link",
                            "r_gripper_palm_link",
                            "r_gripper_r_finger_link",
                            "r_gripper_l_finger_link"]
    links = [find_link(mech, name) for name in rarm_coll_link_names]
    return links
end

function larm_collision_links(mech::Mechanism{PR2})
    larm_coll_link_names = ["l_upper_arm_link",
                            "l_forearm_link",
                            "l_gripper_palm_link",
                            "l_gripper_r_finger_link",
                            "l_gripper_l_finger_link"]
    links = [find_link(mech, name) for name in larm_coll_link_names]
    return links
end

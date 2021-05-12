@testset "invese kinematics_fetch_robot" begin
    urdf_path = Kinematics.__skrobot__.data.fetch_urdfpath()
    for with_base in [false, true]
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

        link = find_link(mech, "gripper_link")
        target_pose = Transform(Kinematics.SVector3f(0.3, -0.4, 1.2))
        q_goal, status = inverse_kinematics!(mech, link, joints, target_pose; with_rot=true)
        @test status == :FTOL_REACHED
        set_joint_angles(mech, joints, q_goal)
        pose_now = get_transform(mech, link)
        @test isapprox(rpy(pose_now), rpy(target_pose), atol=1e-3)
        @test isapprox(translation(pose_now), translation(target_pose), atol=1e-3)
    end
end

@testset "inverse kinematics_pr2" begin
    urdf_path = "../data/fridge.urdf"
    fridge = parse_urdf(urdf_path, with_base=true)
    sdf = UnionSDF(fridge)

    @testset "no collision" begin
        for with_base in [false, true]
            robot = load_pr2(with_base=with_base)
            link = find_link(robot, "l_gripper_tool_frame")
            joints = vcat(rarm_joints(robot), larm_joints(robot))
            reset_manip_pose(robot)
            pose_target = Transform(SVector3f([0.6, 0.7, 0.8]))
            for with_rot in [false, true]
                println(with_rot)
                _, status = inverse_kinematics!(robot, link, joints, pose_target; with_rot=with_rot, ftol=1e-7)
                @test status == :FTOL_REACHED
                pose_actual = get_transform(robot, link)
                pos_diff = translation(pose_target) - translation(pose_actual)
                @test norm(pos_diff) < 1e-3
                if with_rot && with_base 
                    pose2angles(pose) = (tmp = RotZYX(rotation(pose)); [tmp.theta1, tmp.theta2, tmp.theta3])
                    rot_diff = pose2angles(pose_target) - pose2angles(pose_actual)
                    @test norm(rot_diff) < 1e-3
                end
            end
        end
    end
end

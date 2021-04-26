@testset "planning" begin

    function planning_test(with_base)
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

        pose = Transform(Kinematics.SVector3f(0.4, -0.3, 0.7))
        width = Kinematics.SVector3f(0.15, 0.15, 0.5)
        boxsdf = BoxSDF(pose, width)

        n_wp = 30
        q_start = get_joint_angles(mech, joints)

        link = find_link(mech, "gripper_link")
        q_goal = point_inverse_kinematics(mech, link, joints, SVector{3, Float64}(0.3, -0.4, 1.2))
        set_joint_angles(mech, joints, q_goal)

        xi_init = Kinematics.create_straight_trajectory(q_start, q_goal, n_wp)
        q_seq = plan_trajectory(sscc, joints, boxsdf, q_start, q_goal, n_wp)

        for i in 1:n_wp 
            set_joint_angles(mech, joints, q_seq[:, i])
            vals = compute_coll_dists(sscc, joints, boxsdf)
            @test all(vals.>-1e-2, dims=1)[1]
        end
    end

    planning_test(true)
    planning_test(false)
end

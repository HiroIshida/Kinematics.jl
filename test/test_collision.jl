@testset "collision" begin
    function collision_test(with_base)
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


        link_attach = find_link(mech, "wrist_flex_link")
        sscc = SweptSphereCollisionChecker(mech)
        add_coll_links(sscc, link_attach)

        pose = Transform(Kinematics.SVector3f(1., 0., 0.8))
        width = Kinematics.SVector3f(0.3, 0.3, 0.3)
        boxsdf = BoxSDF(pose, width)

        eps = 1e-7
        angles_solved = [0.026928521116837873, 0.2378996102914415, 0.6445784881862138, -0.24833437463054583, -1.035118222590030, -0.170439396116480, -1.3891477169766988, -0.07058932825801573]
        if with_base
            append!(angles_solved, [0., 0, 0])
        end
        set_joint_angles(mech, joints, angles_solved)

        _, grads_analytical = compute_coll_dists_and_grads(sscc, joints, boxsdf)
        dists0 = compute_coll_dists(sscc, joints, boxsdf)
        for i in 1:length(joints)
            av = copy(angles_solved)
            av[i] += eps
            set_joint_angles(mech, joints, av)
            dists1 = compute_coll_dists(sscc, joints, boxsdf)
            @test isapprox(grads_analytical[i, :], (dists1 - dists0)/eps, atol=1e-5)
        end

        @test compute_coll_dists(sscc, joints, boxsdf) == compute_coll_dists_and_grads(sscc, joints, boxsdf)[1]
    end
    collision_test(false)
    collision_test(true)
end

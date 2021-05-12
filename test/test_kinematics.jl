@testset "kinematics" begin
    @testset "forward kinematics" begin
        # kinematics test (testing get_transform)
        f = open("../data/ground_truth.json", "r")
        gtruth_data = JSON.parse(read(f, String))
        close(f)

        urdf_path = Kinematics.__skrobot__.data.pr2_urdfpath()

        function kinematics_test(with_base)
            mech = parse_urdf(urdf_path, with_base=with_base)
            joints = [find_joint(mech, name) for name in gtruth_data["joint_names"]]
            links = [find_link(mech, name) for name in gtruth_data["link_names"]]
            angles = gtruth_data["angle_vector"]
            if with_base
                angles = vcat(angles, [0.3, 0.3, 0.3])
            end
            poses_gtruth = gtruth_data["pose_list"]

            set_joint_angles(mech, joints, angles)
            for i in 1:2
                # check two times to check that cache is propery stored
                for (link, pose_gtruth) in zip(links, poses_gtruth)
                    #println("testing tf ... ")
                    #println(link.name)
                    tf = get_transform(mech, link)
                    if with_base
                        @test translation(tf) ≈ RotZ(mech.base_pose[3]) * pose_gtruth[1:3] + [mech.base_pose[1], mech.base_pose[2], 0.0]
                    else
                        @test translation(tf) ≈ pose_gtruth[1:3]
                    end
                    tmp = RotZYX(rotation(tf))
                    rpy = [tmp.theta3, tmp.theta2, tmp.theta1]
                    ypr = [rpy[3], rpy[2], rpy[1]] # because ypr in test data
                    if with_base
                        @test ypr ≈ pose_gtruth[4:6] + [mech.base_pose[3], 0.0, 0.0]
                    else
                        @test ypr ≈ pose_gtruth[4:6]
                    end
                end
            end

            # kinematics test (testing get_jacobian)
            # TODO test quaternion jacobian
            eps = 1e-7
            angles1 = angles
            angles2 = angles1 * 0 # to test propery cache is deleted

            n_joints = length(joints)
            n_dof = n_joints + (with_base ? 3 : 0)
            for angles in [angles1, angles2]
                for link in mech.links
                    set_joint_angles(mech, joints, angles)
                    J_analytical = get_jacobian(mech, link, joints, true; rpy_jac=true)
                    J_numerical = zeros(6, n_dof)
                    pose0 = get_transform(mech, link)
                    for i in 1:n_dof
                        angles_ = copy(angles)
                        angles_[i] += eps
                        set_joint_angles(mech, joints, angles_)
                        pose1 = get_transform(mech, link)
                        J_numerical[1:3, i] = (translation(pose1) - translation(pose0))/eps
                        J_numerical[4:6, i] = (rpy(pose1) - rpy(pose0))/eps
                    end

                    @test isapprox(J_numerical[1:3, :], J_analytical[1:3, :], atol=1e-5) 
                   
                    # to avoid euler angle's singularity case
                    if norm((J_numerical[4:6, end-2:end])) < 1e4
                        @test isapprox(J_numerical[4:6, :], J_analytical[4:6, :], atol=1e-5) 
                    end
                end
            end
        end
        kinematics_test(false)
        kinematics_test(true)
    end

end

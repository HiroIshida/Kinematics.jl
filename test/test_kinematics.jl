@testset "kinematics" begin
    # kinematics test (testing get_transform)
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
            tmp = RotZYX(rotation(tf))
            rpy = [tmp.theta3, tmp.theta2, tmp.theta1]
            ypr = [rpy[3], rpy[2], rpy[1]] # because ypr in test data
            @test ypr ≈ pose_gtruth[4:6]
        end
    end

    # kinematics test (testing get_jacobian)
    # TODO test quaternion jacobian
    eps = 1e-7
    angles1 = angles
    angles2 = angles1 * 0 # to test propery cache is deleted
    for angles in [angles1, angles2]
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
    end
end

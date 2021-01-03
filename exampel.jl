using Kinematics
urdf_path = Kinematics.__skrobot__.data.fetch_urdfpath()

function main()
    mech = parse_urdf(urdf_path)
    torso_link = find_link(mech, "torso_lift_link")
    function bench()
        for i in 1:100000
            for i in 1:20
                parent_link(mech, torso_link)
            end
        end
    end
    @time bench()
end

main()


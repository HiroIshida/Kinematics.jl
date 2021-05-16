@testset "sdf" begin

    function numerical_gradient(x, f)
        eps = 1e-6
        x0 = Vector{Float64}(x)
        f0 = f(SVector3f(x0))
        grad = zeros(3)
        for i in 1:3
            x1 = copy(x0)
            x1[i] += eps
            grad[i] = (f(SVector3f(x1)) - f0)/eps
        end
        return grad
    end

    @testset "boxsdf" begin
        pose = Transform(SVector3f(0.5, 0.5, 0.5), RotZ(0.3))
        width = SVector3f([1, 1, 1])
        box_sdf = BoxSDF(pose, width)
        @test box_sdf(pose * SVector3f([0.5, 0.5, 0.5])) ≈ 0.0
        @test box_sdf(pose * SVector3f([0.0, 0.0, 0.0])) ≈ -0.5
        @test box_sdf(pose * SVector3f([0.0, 0.0, 1.0])) ≈ 0.5
    end

    @testset "unionsdf" begin
        pose1 = Transform(SVector3f(0.5, 0.5, 0.0))
        pose2 = Transform(SVector3f(-0.5, -0.5, 0.0))
        width = SVector3f([1, 1, 1])
        boxsdf1 = BoxSDF(pose1, width)
        boxsdf2 = BoxSDF(pose2, width)
        unionsdf = UnionSDF([boxsdf1, boxsdf2])
        @test unionsdf(pose1 * SVector3f([0.5, 0.5, 0.5])) ≈ 0.0
        @test unionsdf(pose2 * SVector3f([-0.5, -0.5, -0.5])) ≈ 0.0
        @test unionsdf(pose1 * SVector3f([0.5, 0.5, 1.5])) ≈ 1.0
        @test unionsdf(pose1 * SVector3f([-0.5, -0.5, -1.5])) ≈ 1.0
    end

    @testset "urdfunion_grad" begin
        urdf_path = "../data/fridge.urdf"
        fridge = parse_urdf(urdf_path, with_base=true)
        sdf = UnionSDF(fridge)

        center = [0.0, 0.0, 0.75]
        width = [1.5, 1.5, 1.5]
        point_cloud = [SVector3f(center .- 0.5 * width + width .* rand(3) * 1.5) for i in 1:20]

        grad_out = zeros(3)
        for x in point_cloud
            grad_numel = numerical_gradient(x, (x)->sdf(x))
            sdf(x) # required to save cache  
            Kinematics.gradient!(sdf, x, grad_out)
            @test norm(grad_numel - grad_out) < 1e-4
        end
    end
end

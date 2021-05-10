@testset "sdf" begin
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
end

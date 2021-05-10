@testset "sdf" begin
    @testset "boxsdf" begin
        pose = Transform(SVector3f(0.5, 0.5, 0.5), RotZ(0.3))
        width = SVector3f([1, 1, 1])
        box_sdf = BoxSDF(pose, width)
        @test box_sdf(pose * SVector3f([0.5, 0.5, 0.5])) ≈ 0.0
        @test box_sdf(pose * SVector3f([0.0, 0.0, 0.0])) ≈ -0.5
        @test box_sdf(pose * SVector3f([0.0, 0.0, 1.0])) ≈ 0.5
    end
end

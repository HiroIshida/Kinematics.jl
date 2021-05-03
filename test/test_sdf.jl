@testset "sdf" begin
    pose = Transform(SVector3f(0.5, 0.5, 0.5))
    width = SVector3f([1, 1, 1])
    box_sdf = BoxSDF(pose, width)
    @test box_sdf(SVector3f([0., 0., 0])) == 0.0
    @test box_sdf(SVector3f([1., 1., 1])) == 0.0
    @test box_sdf(SVector3f([0.5, 0.5, 0.5])) == -0.5
    @test box_sdf(SVector3f([2.0, 0., 0.])) == 1.0
    @test box_sdf(SVector3f([-0.5, -0.5, -0.5])) ≈ 0.5 * sqrt(3)
end

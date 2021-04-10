@testset "cache" begin
    cache = CacheVector(3, 1.0)
    @assert typeof(cache) == CacheVector{Float64}
    set_cache!(cache, 2, 2.0)
    @test get_cache(cache, 2)==2.0
    @test !iscached(cache, 1)
    @test iscached(cache, 2)
    @test_throws AssertionError get_cache(cache, 1)
    @test_throws AssertionError set_cache!(cache, 2, 2.0)
    Kinematics.invalidate!(cache)
    @test_throws AssertionError get_cache(cache, 2)
end

@testset "stack" begin
    n_max = 5
    ps = PseudoStack(Int64, n_max)
    for i in 1:n_max
        push!(ps, i)
    end
    @test !isempty(ps)
    @test pop!(ps) == 5
    @test pop!(ps) == 4
    @test pop!(ps) == 3
    @test pop!(ps) == 2
    @test pop!(ps) == 1
    @test isempty(ps)
end

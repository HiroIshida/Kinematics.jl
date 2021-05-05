import Base.*

struct Transform
    mat::SMatrix{4, 4, Float64, 16}
end

@inline function Transform(trans::SVector3f, rot::Rotation{3, Float64})
    R = convert(RotMatrix3{Float64}, rot)
    @inbounds mat = @SMatrix [R[1] R[4] R[7] trans[1];
                              R[2] R[5] R[8] trans[2];
                              R[3] R[6] R[9] trans[3];
                              0 0 0 1]
    Transform(mat)
end

@inline function Transform(rot::Rotation{3, Float64})
    R = convert(RotMatrix3{Float64}, rot)
    @inbounds mat = @SMatrix [R[1] R[4] R[7] 0;
                              R[2] R[5] R[8] 0;
                              R[3] R[6] R[9] 0;
                              0 0 0 1]
    Transform(mat)
end

@inline function Transform(trans::SVector3f)
    @inbounds mat = @SMatrix [1 0 0 trans[1];
                              0 1 0 trans[2];
                              0 0 1 trans[3];
                              0 0 0 1]
    Transform(mat)
end

@inline function base_pose_to_transform(pose::StaticVector{3, T}) where T<:AbstractFloat
    q = UnitQuaternion(cos(0.5 * pose[3]), 0.0, 0.0, sin(0.5 * pose[3]))
    trans = SVector3f(pose[1], pose[2], 0.0)
    return Transform(trans, q)
end

@inline function Base.:*(tf::Transform, point::StaticVector{3, T}) where T<:AbstractFloat
    return translation(tf) + rotation(tf) * point
end
@inline rotation(t::Transform) = @inbounds return RotMatrix(t.mat[1], t.mat[2], t.mat[3], t.mat[5], t.mat[6], t.mat[7], t.mat[9], t.mat[10], t.mat[11])
@inline translation(t::Transform) = @inbounds return SVector(t.mat[13], t.mat[14], t.mat[15])

@inline function rpy(t::Transform)
    @inbounds rpy = RotZYX(rotation(t))
    return [rpy.theta3, rpy.theta2, rpy.theta1]
end

@inline function Base.zero(::Type{Transform})
    Transform(one(SMatrix{4, 4, Float64}))
end

@inline function Base.one(::Type{Transform})
    Transform(one(SMatrix{4, 4, Float64}))
end

function (*)(tf12::Transform, tf23::Transform)
    return Transform(tf12.mat * tf23.mat)
end

function LinearAlgebra.inv(tf::Transform)
    rotinv = inv(rotation(tf))
    Transform(- rotinv * translation(tf), rotinv)
end

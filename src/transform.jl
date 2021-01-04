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
    R = convert(RotMatrix3{Float64}, rot)
    @inbounds mat = @SMatrix [0 0 0 trans[1];
                              0 0 0 trans[2];
                              0 0 0 trans[3];
                              0 0 0 1]
    Transform(mat)
end

@inline rotation(t::Transform) = @inbounds return RotMatrix(t.mat[1], t.mat[2], t.mat[3], t.mat[5], t.mat[6], t.mat[7], t.mat[9], t.mat[10], t.mat[11])
@inline translation(t::Transform) = @inbounds return SVector(t.mat[13], t.mat[14], t.mat[15])

@inline function Base.zero(::Type{Transform})
    Transform(one(SMatrix{4, 4, Float64}))
end

function (*)(tf12::Transform, tf23::Transform)
    return Transform(tf12.mat * tf23.mat)
end

function (*)(tf12::Transform, vec23::SVector3f)
    return Transform(tf12.mat * tf23.mat)
end

"""
@inline function Base.zero(::Type{Transform})
    Transform(zero(SVector{3, Float64}), zero(UnitQuaternion))
end

@inline function Transform(translation::SVector3f, rot::SVector4f)
    Transform(translation, UnitQuaternion(rot))
end

@inline function Transform(translation::SVector3f, rpy::SVector3f)
    Transform(translation, UnitQuaternion(RotXYZ(rpy...)))
end

function Transform(translation::SVector3f, rotmat::AbstractMatrix)
    rotation = UnitQuaternion(rotmat)
    return Transform(translation, rotation)
end

function (*)(tf12::Transform, tf23::Transform)
    rot13 = tf12.rotation * tf23.rotation
    tran13 = tf12.rotation * tf23.translation + tf12.translation
    return Transform(tran13, rot13)
end

function (*)(tf12::Transform, trans23::SVector3f)
    tran13 = tf12.rotation * trans23 + tf12.translation
    return Transform(tran13, tf12.rotation)
end


function (*)(tf12::Transform, rot23::UnitQuaternion)
    return Transform(tf12.translation, tf12.rotation * rot23)
end

(tf::Transform)(vec::SVector3f) = tf.rotation * vec + tf.translation
"""



import Base.*

mutable struct Transform
    translation::SVector3f
    rotation::UnitQuaternion
end

@inline function Base.zero(::Type{Transform})
    Transform(zero(SVector{3, Float64}), zero(UnitQuaternion))
end

"""
strange
@inline function Base.inv(tf::Transform)
    new_rot = inv(tf.rotation)
    new_trans = -new_rot * tf.translation
    Transform(new_trans, new_rot)
end
"""

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

function (*)(tf::Transform, vec::SVector3f)
    return tf.rotation * vec + tf.translation
end



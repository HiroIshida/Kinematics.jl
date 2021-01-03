import Base.*

mutable struct Transform
    translation::SVector3f
    rotation::UnitQuaternion
end
function Transform(pos::SVector3f, rot::SVector4f)
    Transform(pos, UnitQuaternion(rot...))
end

function Transform() # returns identity transform
    Transform(SVector3f([0, 0, 0]), SVector4f([0., 0., 0., 1.]))
end

function Transform(translation::SVector3f, rotmat::AbstractMatrix)
    rotation = UnitQuaternion(rotmat)
    return Transform(translation, rotation)
end

function (*)(tf12::Transform, tf23::Transform)
    rot13 = tf12.rotation * tf23.rotation
    tran13 = tf23.translation + tf23.rotation * tf12.translation
    return Transform(tran13, rot13)
end



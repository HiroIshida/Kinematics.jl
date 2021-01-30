abstract type SignedDistanceFunction end

struct BoxSDF <: SignedDistanceFunction
    pose::Transform
    inv_pose::Transform
    width::SVector3f
end
function BoxSDF(pose::Transform, width::SVector3f)
    BoxSDF(pose, inv(pose), width)
end
function BoxSDF(width::SVector3f)
    BoxSDF(one(Transform), one(Transform), width)
end

function (sdf::BoxSDF)(p::SVector3f)
    half_extent = 0.5 * sdf.width
    p_sdfframe = sdf.inv_pose * p
    q = abs.(p_sdfframe) - half_extent
    return norm(max.(q, 0.0)) + min(maximum(q), 0.0)
end

function get_transform(m::Mechanism, link::Link)
    iscached(m, link) && (return get_cache(m, link))
    return _get_transform(m, link)
end

function _get_transform(m::Mechanism, hlink::Link)
    # hlink : here link
    # plink : parent link
    @debugassert isempty(m.tf_stack)
    @debugassert isempty(m.link_id_stack)

    tf_world_to_hlink = _get_shallowest_cache!(m, hlink)

    while(!isempty(m.tf_stack))
        hlink_id = pop!(m.link_id_stack)
        tf_plink_to_hlink = pop!(m.tf_stack)
        tf_world_to_hlink = tf_world_to_hlink * tf_plink_to_hlink
        set_cache!(m, hlink_id, tf_world_to_hlink)
    end
    return tf_world_to_hlink
end

function _get_shallowest_cache!(m::Mechanism, hlink::Link)
    while(!isroot(hlink))
        iscached(m, hlink) && (return get_cache(m, hlink))
        plink = parent_link(m, hlink)
        hjoint = parent_joint(m, hlink)
        angle = joint_angle(m, hjoint)
        tf_plink_to_hlink = joint_transform(hjoint, angle)

        push!(m.tf_stack, tf_plink_to_hlink)
        push!(m.link_id_stack, hlink.id)
        hlink = parent_link(m, hlink)
    end
    # couldn't find the cache. Thus, returning the transform of the root 
    return m.with_base ? base_pose_to_transform(m.base_pose) : zero(Transform)
end

# TODO cache joint axes
# TODO reduce computation time for joint axis by definining type of axis x, y, z
# which simplifies M * vec to just M[1:3, 1]
function _get_joint_axis(m::Mechanism, hjoint::Joint)
    @debugassert typeof(hjoint) != Joint{Fixed} # fixed joint does not have axis

    iscached(m.axis_cache, hjoint.id) && (return get_cache(m.axis_cache, hjoint.id))

    tf_world_to_plink = get_transform(m, parent_link(m, hjoint))
    tf_world_to_hjoint = tf_world_to_plink * hjoint.pose
    origin = translation(tf_world_to_hjoint)
    axis = rotation(tf_world_to_hjoint) * hjoint.jt.axis
    faxis = FloatingAxis(origin, axis)
    set_cache!(m.axis_cache, hjoint.id, faxis)
    return faxis
end

function rpy_derivative!(rpy::Vector, axis::AbstractVector, out::AbstractVector)
    a1, a2, a3 = -rpy
    x, y, z = axis
    dr_dt = cos(a3)/cos(a2)*x - sin(a3)/cos(a2)*y
    dp_dt = sin(a3)*x + cos(a3)*y
    dy_dt = -cos(a3)*sin(a2)/cos(a2)*x + sin(a3)*sin(a2)/cos(a2)*y + z
    out[:] = [dr_dt, dp_dt, dy_dt]
end

function joint_jacobian!(m::Mechanism, link::Link, joint::Joint{Revolute}, tf_world_to_link, with_rot, mat_out; rpy_jac=false)
    faxis = _get_joint_axis(m, joint)
    diff = cross(faxis.axis, translation(tf_world_to_link) - faxis.origin)
    mat_out[1:3] = diff
    if with_rot
        if rpy_jac
            rpy_derivative!(rpy(tf_world_to_link), faxis.axis, @view mat_out[4:6])
        else
            mat_out[4:6] = faxis.axis
        end
    end
end

function joint_jacobian!(m::Mechanism, link::Link, joint::Joint{Prismatic}, tf_world_to_link, with_rot, mat_out; rpy_jac=false)
    faxis = _get_joint_axis(m, joint)
    mat_out[1:3] = faxis.axis
end

function get_jacobian!(m::Mechanism, link::Link, joints::Vector{<:Joint},
        with_rot::Bool,
        mat_out::AbstractMatrix
        ;
        rpy_jac=false
    )
    tf_world_to_link = get_transform(m, link)
    n_joint = length(joints)
    for i in 1:n_joint
        joint = joints[i]
        if is_relevant(m, joint, link)
            joint_jacobian!(m, link, joint, tf_world_to_link, with_rot, @view mat_out[:, i]; rpy_jac=rpy_jac)
        end
    end

    if m.with_base
        trans_from_root = translation(tf_world_to_link) - [m.base_pose[1], m.base_pose[2], 0.0]
        x, y, _ = trans_from_root
        mat_out[1:3, n_joint+1:n_joint+3] = [1. 0 -y; 0 1 x; 0 0 0]
        if with_rot
            mat_out[4:6, n_joint+1:n_joint+3] = [0. 0 0; 0 0 0; 0 0 1.0]
        end
    end
end

function get_jacobian(m::Mechanism, link::Link, joints::Vector{<:Joint}, with_rot::Bool; rpy_jac=false)
    rows = (with_rot ? 6 : 3)
    cols = (m.with_base ? length(joints) + 3 : length(joints))
    jacobian = zeros(Float64, rows, cols)
    get_jacobian!(m, link, joints, with_rot, jacobian; rpy_jac=rpy_jac)
    return jacobian
end

function point_inverse_kinematics_nakamura(m::Mechanism, link::Link, joints::Vector{<:Joint}, point_desired::SVector3f)
    n_dof = length(joints)
    jac = zero(SizedMatrix{3, n_dof, Float64}) # pre allocate this
    angles = zero(SizedVector{n_dof, Float64})
    get_joint_angles!(m, joints, angles)
    for i in 1:50
        set_joint_angles(m, joints, angles)
        point_now = translation(get_transform(m, link))
        get_jacobian!(m, link, joints, false, jac)
        sr_weight = 1.0
        jac_sharp = transpose(jac)*inv(jac * transpose(jac) .+ sr_weight)
        point_diff = point_desired - point_now
        angles += jac_sharp * point_diff
    end
    return angles
end

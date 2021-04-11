function get_transform(m::Mechanism, link::Link)
    iscached(m, link) && (return get_cache(m, link))
    return _get_transform(m, link)
end

function _get_transform(m::Mechanism, hlink::Link)
    # hlink : here link
    # plink : parent link
    @debugassert isempty(m.tf_stack)
    @debugassert isempty(m.link_id_stack)
    tf_world_to_hlink = zero(Transform) # TODO with_base
    while(!isroot(hlink))
        if iscached(m, hlink) 
            tf_world_to_hlink = get_cache(m, hlink)
            break
        end
        plink = parent_link(m, hlink)
        hjoint = parent_joint(m, hlink)
        angle = joint_angle(m, hjoint)
        tf_plink_to_hlink = joint_transform(hjoint, angle)

        push!(m.tf_stack, tf_plink_to_hlink)
        push!(m.link_id_stack, hlink.id)
        hlink = parent_link(m, hlink)
    end

    while(!isempty(m.tf_stack))
        hlink_id = pop!(m.link_id_stack)
        tf_plink_to_hlink = pop!(m.tf_stack)
        tf_world_to_hlink = tf_world_to_hlink * tf_plink_to_hlink
        set_cache!(m, hlink_id, tf_world_to_hlink)
    end
    return tf_world_to_hlink
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

function joint_jacobian!(m::Mechanism, link::Link, joint::Joint{Revolute}, tf_world_to_link, with_rot, mat_out)
    faxis = _get_joint_axis(m, joint)
    diff = cross(faxis.axis, translation(tf_world_to_link) - faxis.origin)
    mat_out[1:3] = diff
    if with_rot
        mat_out[4:6] = faxis.axis
    end
end

function joint_jacobian!(m::Mechanism, link::Link, joint::Joint{Prismatic}, tf_world_to_link, with_rot, mat_out)
    faxis = _get_joint_axis(m, joint)
    mat_out[1:3] = faxis.axis
end

function get_jacobian!(m::Mechanism, link::Link, joints::Vector{J},
        with_rot::Bool,
        mat_out::AbstractMatrix) where J<:Joint
    tf_world_to_link = get_transform(m, link)
    for i in 1:length(joints)
        joint = joints[i]
        if is_relevant(m, joint, link)
            joint_jacobian!(m, link, joint, tf_world_to_link, with_rot, @view mat_out[:, i])
        end
    end
end

function get_jacobian(m::Mechanism, link::Link, joints::Vector{Joint}, with_rot::Bool)
    rows = (with_rot ? 6 : 3)
    jacobian = zeros(Float64, rows, length(joints))
    get_jacobian!(m, link, joints, with_rot, jacobian)
    return jacobian
end

function point_inverse_kinematics_nakamura(m::Mechanism, link::Link, joints::Vector{Joint}, point_desired::SVector3f)
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

function point_inverse_kinematics(m::Mechanism, link::Link, joints::Vector{Joint}, point_desired::SVector3f; ftol=1e-3)
    n_dof = length(joints)
    jac = zero(SizedMatrix{3, n_dof, Float64}) # pre allocate this

    function f_objective(angles::Vector, grad::Vector)
        set_joint_angles(m, joints, angles)
        point_now = translation(get_transform(m, link))
        point_diff = point_desired - point_now

        get_jacobian!(m, link, joints, false, jac)
        grad_ = - 2 * transpose(jac) * point_diff
        if length(grad) > 0
            for i in 1:n_dof
                grad[i] = grad_[i]
            end
        end
        return sum(point_diff.^2)
    end

    angles_current = get_joint_angles(m, joints)
    opt = Opt(:LD_SLSQP, n_dof)
    opt.min_objective = f_objective
    joint_lower_limits = [lower_limit(j) for j in joints]
    joint_upper_limits = [upper_limit(j) for j in joints]
    opt.lower_bounds = joint_lower_limits
    opt.upper_bounds = joint_upper_limits
    opt.ftol_abs = ftol
    minf, minx, ret = NLopt.optimize(opt, angles_current)
    return minx
end

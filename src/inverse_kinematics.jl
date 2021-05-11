
function inverse_kinematics!(m::Mechanism, link::Link, joints::Vector{<:Joint}, target_pose::Transform; 
        ftol=1e-5, 
        with_rot=true, 
        sscc::Union{Nothing, SweptSphereCollisionChecker}=nothing,
        sdf::Union{Nothing, <:AbstractSDF}=nothing
    )
    n_dof = length(joints) + (m.with_base ? 3 : 0)
    n_rows = (with_rot ? 6 : 3)
    jac = zero(SizedMatrix{n_rows, n_dof, Float64}) # pre allocate this

    function f_objective(angles::Vector, grad::Vector)
        set_joint_angles(m, joints, angles)
        pose_now = get_transform(m, link)
        point_diff = translation(target_pose) - translation(pose_now)
        rpy_diff = rpy(target_pose) - rpy(pose_now)
        pose_diff = (with_rot ? vcat(point_diff, rpy_diff) : point_diff)

        get_jacobian!(m, link, joints, with_rot, jac; rpy_jac=true)
        grad_ = -2 * transpose(jac) * pose_diff

        length(grad) > 0 && copy!(grad, grad_)
        return sum(pose_diff.^2)
    end


    angles_current = get_joint_angles(m, joints)
    opt = Opt(:LD_SLSQP, n_dof)
    opt.min_objective = f_objective
    joint_lower_limits = [lower_limit(j) for j in joints]
    joint_upper_limits = [upper_limit(j) for j in joints]
    if m.with_base
        append!(joint_lower_limits, [-Inf, -Inf, -Inf])
        append!(joint_upper_limits, [Inf, Inf, Inf])
    end
    opt.lower_bounds = joint_lower_limits
    opt.upper_bounds = joint_upper_limits
    if sscc!=nothing && sdf!=nothing
        G = IneqConst(sscc, joints, sdf, 1, 0.02)
        inequality_constraint!(opt, nloptize(G), [1e-8 for _ in 1:G.n_cons])
    end
    opt.ftol_abs = ftol
    minf, minx, ret = NLopt.optimize(opt, angles_current)
    return minx
end

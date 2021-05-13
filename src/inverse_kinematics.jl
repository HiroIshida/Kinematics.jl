function inverse_kinematics!(m::Mechanism, link::Link, joints::Vector{<:Joint}, target_pose::Transform, 
        sscc::SweptSphereCollisionChecker, sdf::AbstractSDF;
        use_bistage=true,
        ftol=1e-5, 
        with_rot=true,
    )

    if use_bistage
        # solve collsion-free problem first, and then use the obtained configuration as the seed
        opt = inverse_kinematics_problem(m, link, joints, target_pose; ftol=ftol, with_rot=with_rot)
        angles_current = get_joint_angles(m, joints)
        minf, minx, ret = NLopt.optimize(opt, angles_current)
    end

    opt = inverse_kinematics_problem(m, link, joints, target_pose; ftol=ftol, with_rot=with_rot)
    G = IneqConst(sscc, joints, sdf, 1, 0.02)
    inequality_constraint!(opt, nloptize(G), [1e-8 for _ in 1:G.n_cons])
    angles_current = get_joint_angles(m, joints)
    minf, minx, ret = NLopt.optimize(opt, angles_current)
    return minx, ret
end

function inverse_kinematics!(m::Mechanism, link::Link, joints::Vector{<:Joint}, target_pose::Transform; 
        ftol=1e-5, 
        with_rot=true) 
    opt = inverse_kinematics_problem(m, link, joints, target_pose; ftol=ftol, with_rot=with_rot)
    angles_current = get_joint_angles(m, joints)
    minf, minx, ret = NLopt.optimize(opt, angles_current)
    return minx, ret
end

function inverse_kinematics_problem(m::Mechanism, link::Link, joints::Vector{<:Joint}, target_pose::Transform; 
        ftol=1e-5, with_rot=true)
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
    opt.ftol_abs = ftol
    return opt
end

mutable struct Objective
    A::SparseMatrixCSC{Float64, Int64}
    n_dim::Int
end

function Objective(n_wp::Int, weights)
    # create cost matrix A
    acc_block = [1 -2 1; -2 4 -2; 1 -2 1]
    A_sub = zeros(n_wp, n_wp)
    for i in 2:n_wp-1
        A_sub[i-1:i+1, i-1:i+1] += acc_block
    end
    weight_matrix = Diagonal(weights.^2)
    A = kron(A_sub, weight_matrix)
    n_dim = size(A, 2)
    Objective(sparse(A), n_dim)
end

function (this::Objective)(xi::AbstractVector, grad::AbstractVector)
    tmp = this.A * xi
    val = transpose(xi) * tmp
    length(grad) > 0 && copy!(grad, 2 * tmp)
    return val
end

function create_straight_trajectory(q_start, q_goal, n_wp)
    interval = (q_goal - q_start) / (n_wp - 1)
    xi = vcat([q_start + interval * (i-1) for i in 1:n_wp]...)
    return xi
end

function convertto_nlopt_const(const_canonical)
    function inner(val::Vector, x::Vector, jac::Matrix)
        val_, jac_ = const_canonical(x)
        copy!(val, -val_)
        length(jac) > 0 && copy!(jac, -transpose(jac_))
    end
    return inner
end

function construct_problem(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{Joint},
        sdf::SignedDistanceFunction,
        q_start, q_goal, n_wp, n_dof
        )
    n_whole = n_dof * n_wp

    function create_ineqconst()
        n_coll = length(sscc.sphere_links)

        # declare beforehand to avoid additional allocation 
        n_ineq = n_coll * n_wp
        jac_mat = zeros(n_ineq, n_whole)
        val_vec = zeros(n_ineq)
        function ineqconst(xi::Vector)
            xi_reshaped = reshape(xi, (n_dof, n_wp))
            for i in 1:n_wp
                angles = xi_reshaped[:, i]
                set_joint_angles(sscc.mech, joints, angles)
                dists, grads = compute_coll_dists_and_grads(sscc, joints, sdf)

                # jac_mat has a block diagonal structure # TODO use BlockArray?
                jac_mat[1+n_coll*(i-1):n_coll*i, 1+n_dof*(i-1):n_dof*i] = transpose(grads)
                val_vec[1+n_coll*(i-1):n_coll*i] = dists
            end
            return val_vec, jac_mat
        end
        return ineqconst, n_ineq
    end

    function create_eqconst()
        # n_dof * 2 means sum of dofs of start and end points
        # the jac_mat is static, so defined offline here
        n_eq = n_dof * 2
        jac_mat = zeros(n_eq, n_whole) 
        jac_mat[1:n_dof, 1:n_dof] = -Matrix{Float64}(I, n_dof, n_dof)
        jac_mat[n_dof+1:end, end-n_dof+1:end] = -Matrix{Float64}(I, n_dof, n_dof)

        val_vec = zeros(n_dof * 2)
        function eqconst(xi::Vector)
            xi_reshaped = reshape(xi, (n_dof, n_wp))
            val_vec[1:n_dof] = q_start - xi_reshaped[:, 1]
            val_vec[n_dof+1:end] = q_goal - xi_reshaped[:, end]
            return val_vec, jac_mat
        end
        return eqconst, n_eq
    end

    f = Objective(n_wp, ones(n_dof))
    g, n_ineq = create_ineqconst()
    h, n_eq = create_eqconst()
    return f, g, h, n_whole, n_ineq, n_eq

end

function plan_trajectory(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{Joint},
        sdf::SignedDistanceFunction,
        q_start, q_goal, n_wp
        ;
        ftol_abs=1e-3,
        solver=:NLOPT
        )
    n_dof = length(joints) + (sscc.mech.with_base ? 3 : 0)
    @assert length(q_start) == n_dof
    @assert length(q_goal) == n_dof

    set_joint_angles(sscc.mech, joints, q_start)
    @assert all(compute_coll_dists(sscc, joints, sdf) .> 0.0, dims=1)[1]

    set_joint_angles(sscc.mech, joints, q_goal)
    @assert all(compute_coll_dists(sscc, joints, sdf) .> 0.0, dims=1)[1]

    xi_init = create_straight_trajectory(q_start, q_goal, n_wp)
    f, g, h, n_whole, n_ineq, n_eq = construct_problem(sscc, joints, sdf, q_start, q_goal, n_wp, n_dof)

    if solver==:NLOPT
        opt = Opt(:LD_SLSQP, n_whole)
        opt.min_objective = (x::Vector, grad::Vector) -> f(x, grad)
        inequality_constraint!(opt, convertto_nlopt_const(g), [1e-8 for _ in 1:n_ineq])
        equality_constraint!(opt, convertto_nlopt_const(h), [1e-8 for _ in 1:n_eq])
        opt.ftol_abs = ftol_abs
        minf, xi_solved, ret = NLopt.optimize(opt, xi_init)
        ret == :FORCED_STOP && error("nlopt forced stop")
    else
        error("not an available solver")
    end

    q_seq = reshape(xi_solved, (n_dof, n_wp))
    return q_seq, ret
end

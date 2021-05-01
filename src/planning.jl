mutable struct Objective
    A::SparseMatrixCSC{Float64, Int64}
    n_dim::Int
    _tmp::Vector # to avoid alloc
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
    _tmp = zeros(n_dim)
    Objective(sparse(A), n_dim, _tmp)
end

function (this::Objective)(xi::AbstractVector, grad::AbstractVector)
    tmp = this._tmp
    mul!(tmp, this.A, xi)
    val = transpose(xi) * tmp
    length(grad) > 0 && mul!(grad, 2, tmp)
    return val
end

abstract type Constraint end

mutable struct IneqConst <: Constraint
    sscc::SweptSphereCollisionChecker
    joints::Vector{Joint}
    sdf::SignedDistanceFunction
    n_wp::Int
    n_dof::Int
    n_coll::Int
    n_cons::Int

    # uesd only in NLOPT
    jac_mat::Matrix{Float64}
    val_vec::Vector{Float64}
end
function IneqConst(sscc::SweptSphereCollisionChecker, joints, sdf, n_wp)
    n_dof = length(joints) + (sscc.mech.with_base ? 3 : 0)
    n_coll = length(sscc.sphere_links)
    n_cons = n_coll * n_wp
    jac_mat = zeros(n_dof * n_wp, n_cons)
    val_vec = zeros(n_cons)
    return IneqConst(sscc, joints, sdf, n_wp, n_dof, n_coll, n_cons, jac_mat, val_vec)
end

function (this::IneqConst)(xi::AbstractVector, val_vec::AbstractVector, jac_mat::AbstractMatrix)
    n_dof, n_wp, n_coll, n_cons = this.n_dof, this.n_wp, this.n_coll, this.n_cons
    xi_reshaped = reshape(xi, (n_dof, n_wp))
    for i in 1:n_wp
        angles = xi_reshaped[:, i]
        set_joint_angles(this.sscc.mech, this.joints, angles)
        dists, grads = compute_coll_dists_and_grads(this.sscc, this.joints, this.sdf)

        # jac_mat has a block diagonal structure # TODO use BlockArray?
        jac_mat[1+n_dof*(i-1):n_dof*i, 1+n_coll*(i-1):n_coll*i] = grads
        val_vec[1+n_coll*(i-1):n_coll*i] = dists
    end
end

mutable struct EqConst <: Constraint
    n_dof::Int
    n_wp::Int
    n_cons::Int
    q_start::Vector{Float64}
    q_goal::Vector{Float64}

    # uesd only in NLOPT
    jac_mat::Matrix{Float64}
    val_vec::Vector{Float64}
end
function EqConst(n_dof, n_wp, q_start, q_goal)
    n_cons = n_dof * 2
    n_whole = n_dof * n_wp
    jac_mat = zeros(n_whole, n_cons)
    val_vec = zeros(n_cons)
    EqConst(n_dof, n_wp, n_cons, q_start, q_goal, jac_mat, val_vec)
end

function (this::EqConst)(xi::AbstractVector, val_vec::AbstractVector, jac_mat::AbstractMatrix)
    n_dof, n_wp = this.n_dof, this.n_wp
    xi_reshaped = reshape(xi, (n_dof, n_wp))
    jac_mat[1:n_dof, 1:n_dof] = -Matrix{Float64}(I, n_dof, n_dof)
    jac_mat[end-n_dof+1:end, n_dof+1:end] = -Matrix{Float64}(I, n_dof, n_dof)
    val_vec[1:n_dof] = this.q_start - xi_reshaped[:, 1]
    val_vec[n_dof+1:end] = this.q_goal - xi_reshaped[:, end]
end

function nloptize(cons::Constraint)
    function inner(val::Vector, xi::Vector, jac::Matrix)
        cons(xi, cons.val_vec, cons.jac_mat)
        copy!(val, -cons.val_vec)
        length(jac) > 0 && copy!(jac, -cons.jac_mat)
    end
    return inner
end

mutable struct IpoptManager
    objective::Objective
    ineqconst::IneqConst
    eqconst::EqConst

    idxes::Vector{Tuple{Int, Int}}

    f_grad_cache::Vector{Float64}
    g_jac_cache::Matrix{Float64} # vector because jac is sparse matrix
end

function IpoptManager(objective::Objective, ineqconst::IneqConst, eqconst::EqConst) 
    n_cons_total = eqconst.n_cons + ineqconst.n_cons
    n_decision = eqconst.n_dof * eqconst.n_wp
    jac_mat = ones(n_decision, n_cons_total) * Inf # actually, transponse of jac
    
    # adhoc
    jac_mat_ineq = @view jac_mat[:, 1:ineqconst.n_cons]
    jac_mat_eq = @view jac_mat[:, ineqconst.n_cons+1:end]
    x_dummy = zeros(n_decision)
    val_dummy_ineq = ones(ineqconst.n_cons)
    val_dummy_eq = ones(eqconst.n_cons)

    ineqconst(x_dummy, val_dummy_ineq, jac_mat_ineq)
    eqconst(x_dummy, val_dummy_eq, jac_mat_eq)
    idxes = Tuple{Int, Int}[]
    for j in 1:size(jac_mat, 1)
        for i in 1:size(jac_mat, 2)
            if jac_mat[j, i] != Inf
                push!(idxes, (i, j))
            end
        end
    end

    f_grad_cache = ones(n_decision) * Inf
    g_jac_cache = ones(length(idxes)) * Inf
    IpoptManager(objective, ineqconst, eqconst, idxes, f_grad_cache, jac_mat)
end

eval_f(ipoptm::IpoptManager, x) = ipoptm.objective(x, ipoptm.f_grad_cache)
function eval_f_grad(ipoptm::IpoptManager, x, grad)
    eval_f(ipoptm, x) # TODO ad hoc. Please consider removing this 
    grad[:] = ipoptm.f_grad_cache
end
#eval_f_grad(ipoptm::IpoptManager, x, grad) = (grad[:] = ipoptm.f_grad_cache; println(grad))
function eval_g(ipoptm::IpoptManager, x, val_vec)
    n_cons_total = ipoptm.eqconst.n_cons + ipoptm.ineqconst.n_cons
    n_decision = ipoptm.eqconst.n_dof * ipoptm.eqconst.n_wp
    jac_mat_ineq = @view ipoptm.g_jac_cache[:, 1:ipoptm.ineqconst.n_cons]
    jac_mat_eq = @view ipoptm.g_jac_cache[:, ipoptm.ineqconst.n_cons+1:end]
    val_vec_ineq = @view val_vec[1:ipoptm.ineqconst.n_cons]
    val_vec_eq = @view val_vec[ipoptm.ineqconst.n_cons+1:end]

    ipoptm.ineqconst(x, val_vec_ineq, jac_mat_ineq)
    ipoptm.eqconst(x, val_vec_eq, jac_mat_eq)
end

function eval_g_jac(ipoptm::IpoptManager, x, mode, rows, cols, values)
    n_elems = length(ipoptm.idxes)
    if mode==:Structure
        for (i, pair) in zip(1:n_elems, ipoptm.idxes)
            rows[i], cols[i] = pair 
        end
    else
        for i in 1:n_elems
            pair = ipoptm.idxes[i]
            values[i] = ipoptm.g_jac_cache[pair[2], pair[1]] # NOTE transposed
        end
    end
end

function create_problem(ipoptm::IpoptManager)
    infty = 1e10
    n = ipoptm.ineqconst.n_dof * ipoptm.ineqconst.n_wp
    x_L = [-infty for _ in 1:n]
    x_U = [infty for _ in 1:n]

    m_ineq = ipoptm.ineqconst.n_cons
    m_eq = ipoptm.eqconst.n_cons
    m = m_ineq + m_eq
    g_L = vcat([0.0 for _ in 1:m_ineq], [0.0 for _ in 1:m_eq])
    g_U = vcat([infty for _ in 1:m_ineq], [0.0 for _ in 1:m_eq])

    nelem_jac = length(ipoptm.idxes)
    nelem_hess = 0 # dummy

    _eval_f = (x)->eval_f(ipoptm, x)
    _eval_f_grad = (x, grad)->eval_f_grad(ipoptm, x, grad)
    _eval_g = (x, val_vec)->eval_g(ipoptm, x, val_vec)
    _eval_g_jac = (x, mode, rows, cols, values) -> eval_g_jac(ipoptm, x, mode, rows, cols, values)
    _eval_h = nothing
    prob = createProblem(n, x_L, x_U, m, g_L, g_U, nelem_jac, nelem_hess, _eval_f, _eval_g, _eval_f_grad, _eval_g_jac, _eval_h)
    addOption(prob, "hessian_approximation", "limited-memory")
    addOption(prob, "print_level", 3) # 1 ~ 12; 12 is most verbose
    return prob
end

function create_straight_trajectory(q_start, q_goal, n_wp)
    interval = (q_goal - q_start) / (n_wp - 1)
    xi = vcat([q_start + interval * (i-1) for i in 1:n_wp]...)
    return xi
end

function construct_problem(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{<:Joint},
        sdf::SignedDistanceFunction,
        q_start, q_goal, n_wp, n_dof
        )
    n_whole = n_dof * n_wp

    F = Objective(n_wp, ones(n_dof))
    G = IneqConst(sscc, joints, sdf, n_wp)
    H = EqConst(n_dof, n_wp, q_start, q_goal)
    return F, G, H, n_whole

end

function plan_trajectory(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{<:Joint},
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
    F, G, H, n_whole = construct_problem(sscc, joints, sdf, q_start, q_goal, n_wp, n_dof)

    joint_lower_limits = [lower_limit(j) for j in joints]
    joint_upper_limits = [upper_limit(j) for j in joints]
    if sscc.mech.with_base
        append!(joint_lower_limits, -[Inf, Inf, Inf])
        append!(joint_upper_limits, [Inf, Inf, Inf])
    end
    lower_bounds = vcat([joint_lower_limits for i in 1:n_wp]...)
    upper_bounds = vcat([joint_upper_limits for i in 1:n_wp]...)

    if solver==:NLOPT
        opt = Opt(:LD_SLSQP, n_whole)
        opt.min_objective = (x::Vector, grad::Vector) -> F(x, grad)
        opt.lower_bounds = lower_bounds
        opt.upper_bounds = upper_bounds
        inequality_constraint!(opt, nloptize(G), [1e-3 for _ in 1:G.n_cons])
        equality_constraint!(opt, nloptize(H), [1e-3 for _ in 1:H.n_cons])
        opt.ftol_abs = ftol_abs
        minf, xi_solved, ret = NLopt.optimize(opt, xi_init)
        ret == :FORCED_STOP && error("nlopt forced stop")
    elseif solver==:IPOPT
        @warn "Solving via IPOPT is still an experimetal feature."
        ipoptm = IpoptManager(F, G, H)
        prob = create_problem(ipoptm)
        prob.x = xi_init
        status = solveProblem(prob)
        ret = status
        xi_solved = prob.x
    else
        error("not an available solver")
    end

    q_seq = reshape(xi_solved, (n_dof, n_wp))
    return q_seq, ret
end

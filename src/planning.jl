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
    margin::Float64

    # uesd only in NLOPT
    jac_mat::Matrix{Float64}
    val_vec::Vector{Float64}
end
function IneqConst(sscc::SweptSphereCollisionChecker, joints, sdf, n_wp, margin)
    n_dof = length(joints) + (sscc.mech.with_base ? 3 : 0)
    n_coll = length(sscc.sphere_links)
    n_cons = n_coll * n_wp
    jac_mat = zeros(n_dof * n_wp, n_cons)
    val_vec = zeros(n_cons)
    return IneqConst(sscc, joints, sdf, n_wp, n_dof, n_coll, n_cons, margin, jac_mat, val_vec)
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
        val_vec[1+n_coll*(i-1):n_coll*i] = dists .- this.margin
    end
end

abstract type PartialConstraint end

struct ConfigurationConstraint <: PartialConstraint
    idx_wp::Int
    n_dof::Int
    n_cons::Int
    q_const::Vector{Float64}
end
function ConfigurationConstraint(idx_wp, n_dof, q_const)
    n_cons = n_dof
    ConfigurationConstraint(idx_wp, n_dof, n_cons, q_const)
end

function (this::ConfigurationConstraint)(q::AbstractVector, val_vec::AbstractVector, jac_mat::AbstractMatrix)
    j_start = 1 + (this.idx_wp - 1) * this.n_dof
    j_end = j_start + this.n_dof - 1
    jac_mat[j_start:j_end, :] = -Matrix{Float64}(I, this.n_dof, this.n_dof)
    val_vec[:] = this.q_const - q
end

struct PoseConstraint <: PartialConstraint
    idx_wp::Int
    n_dof::Int
    n_cons::Int
    move_links::Vector{Link}
    target_poses::Vector{Transform}
    with_rots::Vector{Bool}
    mech::Mechanism
    joints::Vector{Joint}
end

function PoseConstraint(idx_wp::Int, n_dof::Int, 
        move_links::Vector{Link}, target_poses::Vector{Transform}, with_rots::Vector{Bool}, 
        mech::Mechanism, joints::Vector{<:Joint})

    n_cons = sum((pred ? 6 : 3) for pred in with_rots)
    PoseConstraint(idx_wp, n_dof, n_cons, move_links, target_poses, with_rots, mech, joints)
end

function PoseConstraint(idx_wp::Int, n_dof::Int, 
        move_link::Link, target_pose::Transform, with_rot::Bool, 
        mech::Mechanism, joints::Vector{<:Joint})
    PoseConstraint(idx_wp, n_dof, [move_link], [target_pose], [with_rot], mech, joints)
end

function (this::PoseConstraint)(q::AbstractVector, val_vec::AbstractVector, jac_mat::AbstractMatrix)
    mech = this.mech
    joints = this.joints

    set_joint_angles(mech, joints, q)

    j_start = 1 + (this.idx_wp - 1) * this.n_dof
    j_end = j_start + this.n_dof - 1

    i_end = 0
    for (link, target_pose, with_rot) in zip(this.move_links, this.target_poses, this.with_rots)
        current_pose = get_transform(mech, link)
        point_diff = translation(current_pose) - translation(target_pose)
        rpy_diff = rpy(current_pose) - rpy(target_pose)
        pose_diff = (with_rot ? vcat(point_diff, rpy_diff) : point_diff)

        dim = (with_rot ? 6 : 3)
        i_start = i_end + 1
        i_end = i_start + dim - 1

        val_vec[i_start:i_end] = pose_diff
        jac = @view jac_mat[j_start:j_end, i_start:i_end]
        get_jacobian!(this.mech, link, this.joints, with_rot, transpose(jac); rpy_jac=true)
    end
end

mutable struct EqConst <: Constraint
    n_dof::Int
    n_wp::Int
    n_cons::Int
    cons_arr::Vector{PartialConstraint}

    # uesd only in NLOPT
    jac_mat::Matrix{Float64}
    val_vec::Vector{Float64}
end

function EqConst(n_wp, cons_arr::Vector{PartialConstraint})
    @debugassert all(y->y==cons_arr[1].n_dof, (c.n_dof for c in cons_arr))
    n_dof = cons_arr[1].n_dof
    n_cons = sum((c.n_cons for c in cons_arr))

    n_whole = n_dof * n_wp
    jac_mat = zeros(n_whole, n_cons)
    val_vec = zeros(n_cons)
    EqConst(n_dof, n_wp, n_cons, cons_arr, jac_mat, val_vec)
end

function (this::EqConst)(xi::AbstractVector, val_vec::AbstractVector, jac_mat::AbstractMatrix)
    n_dof, n_wp = this.n_dof, this.n_wp
    xi_reshaped = reshape(xi, (n_dof, n_wp))

    i_end = 0
    for cons in this.cons_arr
        i_start = i_end + 1
        i_end = i_start + cons.n_cons - 1 # must followed after i_start!

        q_partial = @view xi_reshaped[:, cons.idx_wp]
        val_vec_partial = @view val_vec[i_start:i_end] 
        jac_mat_partial = @view jac_mat[:, i_start:i_end]
        cons(q_partial, val_vec_partial, jac_mat_partial)
    end
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

    ineqconst(x_dummy, val_dummy_ineq, jac_mat_ineq, margin)
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
        q_start, q_goal, n_wp, n_dof, margin
        ;
        partial_consts=[],
        )
    n_whole = n_dof * n_wp

    eq_cons_arr = PartialConstraint[]
    push!(eq_cons_arr, ConfigurationConstraint(1, n_dof, q_start))
    push!(eq_cons_arr, ConfigurationConstraint(n_wp, n_dof, q_goal))
    append!(eq_cons_arr, partial_consts)

    F = Objective(n_wp, ones(n_dof))
    G = IneqConst(sscc, joints, sdf, n_wp, margin)
    H = EqConst(n_wp, eq_cons_arr)
    return F, G, H, n_whole

end

function plan_trajectory(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{<:Joint},
        sdf::SignedDistanceFunction,
        q_start, q_goal, n_wp
        ;
        margin=2e-2,
        partial_consts=Vector{PartialConstraint}(),
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
    F, G, H, n_whole = construct_problem(sscc, joints, sdf, q_start, q_goal, n_wp, n_dof, margin; partial_consts=partial_consts)

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
        inequality_constraint!(opt, nloptize(G), [1e-8 for _ in 1:G.n_cons])
        equality_constraint!(opt, nloptize(H), [1e-8 for _ in 1:H.n_cons])
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

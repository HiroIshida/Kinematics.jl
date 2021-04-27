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

function (this::IneqConst)(val_vec::AbstractVector, xi::AbstractVector, jac_mat::AbstractMatrix)
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

function (this::EqConst)(val_vec::AbstractVector, xi::AbstractVector, jac_mat::AbstractMatrix)
    n_dof, n_wp = this.n_dof, this.n_wp
    xi_reshaped = reshape(xi, (n_dof, n_wp))
    jac_mat[1:n_dof, 1:n_dof] = -Matrix{Float64}(I, n_dof, n_dof)
    jac_mat[end-n_dof+1:end, n_dof+1:end] = -Matrix{Float64}(I, n_dof, n_dof)
    val_vec[1:n_dof] = this.q_start - xi_reshaped[:, 1]
    val_vec[n_dof+1:end] = this.q_goal - xi_reshaped[:, end]
end

function nloptize(cons::Constraint)
    function inner(val::Vector, xi::Vector, jac::Matrix)
        cons(cons.val_vec, xi, cons.jac_mat)
        copy!(val, -cons.val_vec)
        length(jac) > 0 && copy!(jac, -cons.jac_mat)
    end
    return inner
end

function create_straight_trajectory(q_start, q_goal, n_wp)
    interval = (q_goal - q_start) / (n_wp - 1)
    xi = vcat([q_start + interval * (i-1) for i in 1:n_wp]...)
    return xi
end

function construct_problem(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{Joint},
        sdf::SignedDistanceFunction,
        q_start, q_goal, n_wp, n_dof
        )
    n_whole = n_dof * n_wp

    f = Objective(n_wp, ones(n_dof))
    G = IneqConst(sscc, joints, sdf, n_wp)
    H = EqConst(n_dof, n_wp, q_start, q_goal)
    return f, G, H, n_whole

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
    f, G, H, n_whole = construct_problem(sscc, joints, sdf, q_start, q_goal, n_wp, n_dof)

    H(H.val_vec, xi_init, H.jac_mat)

    if solver==:NLOPT
        opt = Opt(:LD_SLSQP, n_whole)
        opt.min_objective = (x::Vector, grad::Vector) -> f(x, grad)
        inequality_constraint!(opt, nloptize(G), [1e-8 for _ in 1:G.n_cons])
        equality_constraint!(opt, nloptize(H), [1e-8 for _ in 1:H.n_cons])
        opt.ftol_abs = ftol_abs
        minf, xi_solved, ret = NLopt.optimize(opt, xi_init)
        ret == :FORCED_STOP && error("nlopt forced stop")
    else
        error("not an available solver")
    end

    q_seq = reshape(xi_solved, (n_dof, n_wp))
    return q_seq, ret
end

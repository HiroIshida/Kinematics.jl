using SequentialQP

function cost_metric_matrix(n_wp, weights)
    # TODO levarage sparsity ?
    acc_block = [1 -2 1; -2 4 -2; 1 -2 1]
    A_sub = zeros(n_wp, n_wp)
    for i in 2:n_wp-1
        A_sub[i-1:i+1, i-1:i+1] += acc_block
    end
    weight_matrix = Diagonal(weights.^2)
    A = kron(A_sub, weight_matrix)
    return A
end

function create_straight_trajectory(q_start, q_goal, n_wp)
    interval = (q_goal - q_start) / (n_wp - 1)
    xi = vcat([q_start + interval * (i-1) for i in 1:n_wp]...)
    return xi
end

function plan_trajectory(
        sscc::SweptSphereCollisionChecker,
        joints::Vector{Joint},
        sdf::SignedDistanceFunction,
        q_start, q_goal, n_wp
        )
    n_dof = length(joints)
    n_whole = n_dof * n_wp

    function create_objective()
        weights = ones(n_dof)
        A = cost_metric_matrix(n_wp, weights)
        function objective(xi::Vector, grad::Vector) # NLOPT style
            val = transpose(xi) * A * xi
            grad_ = 2 * A * xi
            length(grad) > 0 && copy!(grad, grad_)
            return val
        end
        return objective
    end

    function create_ineqconst()
        n_coll = length(sscc.sphere_links)

        # declare beforehand to avoid additional allocation 
        jac_mat = zeros(n_coll * n_wp, n_whole)
        val_vec = zeros(n_coll * n_wp)
        function ineqconst(xi::Vector, jac::Matrix)
            xi_reshaped = reshape(xi, (n_dof, n_wp))
            for i in 1:n_wp
                angles = xi_reshaped[:, i]
                set_joint_angles(sscc.mech, joints, angles)
                dists, grads = compute_coll_dists_and_grads(sscc, joints, sdf)

                # jac_mat has a block diagonal structure # TODO use BlockArray?
                jac_mat[1+n_coll*(i-1):n_coll*i, 1+n_dof*(i-1):n_dof*i] = transpose(grads)
                val_vec[1+n_coll*(i-1):n_coll*i] = dists
            end
            length(jac) > 0 && copy!(jac, jac_mat)
            return val_vec
        end
        return ineqconst
    end

    function create_eqconst()
        # n_dof * 2 means sum of dofs of start and end points
        # the jac_mat is static, so defined offline here
        jac_mat = zeros(n_dof * 2, n_whole) 
        jac_mat[1:n_dof, 1:n_dof] = -Matrix{Float64}(I, n_dof, n_dof)
        jac_mat[n_dof+1:end, end-n_dof+1:end] = -Matrix{Float64}(I, n_dof, n_dof)

        val_vec = zeros(n_dof * 2)
        function eqconst(xi::Vector, jac::Matrix)
            xi_reshaped = reshape(xi, (n_dof, n_wp))
            val_vec[1:n_dof] = q_start - xi_reshaped[:, 1]
            val_vec[n_dof+1:end] = q_goal - xi_reshaped[:, end]
            length(jac) > 0 && copy!(jac, jac_mat)
            return val_vec
        end
        return eqconst
    end

    xi_init = create_straight_trajectory(q_start, q_goal, n_wp)

    opt = Opt(:LD_SLSQP, n_whole)
    opt.min_objective = create_objective()
    inequality_constraint!(opt, create_ineqconst())
    equality_constraint!(opt, create_eqconst())
    opt.ftol_abs = 1e-3
    minf, xi_solved, ret = NLopt.optimize(opt, xi_init)
    println(ret)
    q_seq = reshape(xi_solved, (n_dof, n_wp))
    return q_seq
end

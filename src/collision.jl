using UUIDs

function collision_trimesh(l::Link)
    if !haskey(l.data, "trimesh")
        if typeof(l.geometric_meta_data) == MeshMetaData
            file_path = l.geometric_meta_data.file_path
            l.data["trimesh"] = __trimesh__.load_mesh(file_path)
        else
            return nothing
        end
    end
    return l.data["trimesh"]
end

function compute_swept_sphere(link::Link)
    trimesh = collision_trimesh(link)
    isnothing(trimesh) && (return Matrix{Float64}(undef, 3, 0), 0.0)

    centers_, radius_ = __skrobot__.planner.swept_sphere.compute_swept_sphere(trimesh)
    n_sphere = size(centers_)[1]

    center_list = Vector{SVector3f}(undef, n_sphere)
    radius_list = Vector{Float64}(undef, n_sphere)
    for i in 1:n_sphere
        center_list[i] = SVector3f(centers_[i, :])
        radius_list[i] = radius_[]
    end
    return center_list, radius_list
end

struct SweptSphereCollisionChecker
    mech::Mechanism
    sphere_links::Vector{Link}
    sphere_radii::Vector{Float64}
end
SweptSphereCollisionChecker(mech::Mechanism) = SweptSphereCollisionChecker(mech, Vector{Link}(undef, 0), Vector{Float64}(undef, 0))

function add_coll_links(sscc::SweptSphereCollisionChecker, coll_link::Link)
    center_list, radius_list = compute_swept_sphere(coll_link)
    for (c, r) in zip(center_list, radius_list)
        link_name = "sphere_" * string(UUIDs.uuid1())
        add_new_link(sscc.mech, coll_link, link_name, c)
        push!(sscc.sphere_links, find_link(sscc.mech, link_name))
        push!(sscc.sphere_radii, r)
    end
end

function add_coll_sphers_to_vis(vis::Visualizer, sscc::SweptSphereCollisionChecker)
    for (sphere, radius) in zip(sscc.sphere_links, sscc.sphere_radii)
        vis_sphere = create_vis_sphere(radius)
        println(sphere.name)
        setobject!(vis[:sscc][sphere.name], vis_sphere, yellow_material())
        settransform!(vis[:sscc][sphere.name], to_affine_map(get_transform(sscc.mech, sphere)))
    end
end

function compute_coll_dists!(sscc::SweptSphereCollisionChecker, joints::Vector{Joint}, sdf::SignedDistanceFunction, 
        out_vals::AbstractArray{Float64, 1})
    for i in 1:length(sscc.sphere_links)
        link = sscc.sphere_links[i]
        pt = translation(get_transform(sscc.mech, link))
        out_vals[i] = sdf(pt)
    end
end

function compute_coll_dists(sscc::SweptSphereCollisionChecker, joints::Vector{Joint}, sdf::SignedDistanceFunction)
    n_feature = length(sscc.sphere_links)
    out_vals = Vector{Float64}(undef, n_feature)
    compute_coll_dists!(sscc, joints, sdf, out_vals)
    return out_vals
end

function compute_coll_dists_and_grads!(sscc::SweptSphereCollisionChecker, joints::Vector{Joint}, sdf::SignedDistanceFunction, 
        out_vals::AbstractArray{Float64, 1}, out_grads::AbstractArray{Float64, 2}
       )
    n_col = length(sscc.sphere_links)
    eps = 1e-7
    grad = MVector{3, Float64}(undef)
    pt1 = MVector{3, Float64}(undef)
    jac = zeros(3, length(joints))

    for i in 1:n_col
        link = sscc.sphere_links[i]
        pt0 = translation(get_transform(sscc.mech, link))
        out_vals[i] = sdf(pt0)

        for j in 1:3
            copy!(pt1, pt0)
            pt1[j] += eps
            grad[j] = (sdf(pt1) - out_vals[i])/eps
        end
        get_jacobian!(sscc.mech, link, joints, false, jac)
        out_grads[:, i] = transpose(grad) * jac
    end
end

function compute_coll_dists_and_grads(sscc::SweptSphereCollisionChecker, joints::Vector{Joint}, sdf::SignedDistanceFunction) 
    n_feature = length(sscc.sphere_links)
    out_vals = Vector{Float64}(undef, n_feature)
    out_grads = Array{Float64}(undef, length(joints), n_feature)
    compute_coll_dists_and_grads!(sscc, joints, sdf, out_vals, out_grads)
    return out_vals, out_grads
end

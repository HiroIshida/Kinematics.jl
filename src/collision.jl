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

function compute_coll_dists!(sscc::SweptSphereCollisionChecker, joints::Vector{Joint}, angles::Vector{Float64}, sdf::SignedDistanceFunction, out_vals::Vector{Float64})
    set_joint_angles(sscc.mech, joints, angles)
    for i in 1:length(sscc.sphere_links)
        link = sscc.sphere_links[i]
        pt = translation(get_transform(sscc.mech, link))
        out_vals[i] = sdf(pt)
    end
end

function compute_coll_dists(sscc::SweptSphereCollisionChecker, joints::Vector{Joint}, angles::Vector{Float64}, sdf::SignedDistanceFunction)
    n_feature = length(sscc.sphere_links)
    out_vals = Vector{Float64}(undef, n_feature)
    compute_coll_dists!(sscc, joints, angles, sdf, out_vals)
    return out_vals
end
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

struct SweptSphereManager
    sphere_link_list::Vector{Link}
    coll_radius_list::Vector{Float64}
end
SweptSphereManager() = SweptSphereManager(Vector{Link}(undef, 0), Vector{Float64}(undef, 0))

function add_collision_link(ssm::SweptSphereManager, mech::Mechanism, coll_link::Link)
    center_list, radius_list = compute_swept_sphere(coll_link)
    for (c, r) in zip(center_list, radius_list)
        link_name = "sphere_" * string(UUIDs.uuid1())
        add_new_link(mech, coll_link, link_name, c)
        push!(ssm.sphere_link_list, find_link(mech, link_name))
        push!(ssm.coll_radius_list, r)
    end
end

function add_collision_spheres(vis::Visualizer, ssm::SweptSphereManager, mech::Mechanism)
    for (sphere, radius) in zip(ssm.sphere_link_list, ssm.coll_radius_list)
        vis_sphere = create_vis_sphere(radius)
        println(sphere.name)
        setobject!(vis[:ssm][sphere.name], vis_sphere)
        settransform!(vis[:ssm][sphere.name], to_affine_map(get_transform(mech, sphere)))
    end
end

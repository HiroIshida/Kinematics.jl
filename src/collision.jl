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
    centers = Matrix{Float64}(transpose(centers_))
    radius = Float64(radius_[])
    return centers, radius
end


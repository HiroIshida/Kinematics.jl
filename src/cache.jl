struct CacheVector{T}
    data::Vector{T}
    iscached_vec::BitVector
end

function CacheVector(n_elem, default_elem)
    data = [deepcopy(default_elem) for i in 1:n_elem]
    iscached_vec = falses(n_elem)
    CacheVector{typeof(default_elem)}(data, iscached_vec)
end

function invalidate!(cv::CacheVector)
    fill!(cv.iscached_vec, false)
    nothing
end

function extend!(cv::CacheVector{T}) where T
    push!(cv.data, zero(T))
    push!(cv.iscached_vec, false)
    nothing
end

@inline function set_cache!(cv::CacheVector{T}, idx, elem::T) where T
    @debugassert cv.iscached_vec[idx] == false
    @inbounds cv.data[idx] = elem
    @inbounds cv.iscached_vec[idx] = true
    nothing
end

@inline function iscached(cv::CacheVector, idx)
    @inbounds return cv.iscached_vec[idx]
end

@inline function get_cache(cv::CacheVector, idx)
    @debugassert cv.iscached_vec[idx] == true
    @inbounds return cv.data[idx]
end


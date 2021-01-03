mutable struct CacheVector{T}
    data::Vector{T}
    iscached_vec::BitVector
end

function CacheVector(n_elem, default_elem)
    data = [deepcopy(default_elem) for i in 1:n_elem]
    iscached_vec = falses(n_elem)
    CacheVector{typeof(default_elem)}(data, iscached_vec)
end

function invalidate!(cv::CacheVector)
    cv.iscached_vec = falses(length(cv.iscached_vec))
    nothing
end

@inline function set_cache!(cv::CacheVector{T}, idx, elem::T) where T
    @debugassert cv.iscached_vec[idx] == false
    cv.data[idx] = elem
    cv.iscached_vec[idx] = true
    nothing
end

@inline function iscached(cv::CacheVector, idx)
    return cv.iscached_vec[idx]
end

@inline function get_cache(cv::CacheVector, idx)
    @debugassert cv.iscached_vec[idx] == true
    return cv.data[idx]
end

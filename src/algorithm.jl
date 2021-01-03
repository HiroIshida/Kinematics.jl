function get_transform(m::Mechanism, link::Link)
    iscached(m.tf_cache, link.id) && (return get_cache(m.tf_cache, link_id))
    return get_transform_recursive(m, link)
end

function get_transform_recursive(m::Mechanism, hlink::Link)
    # hlink : here link
    # plink : parent link
    tf_world_to_root = zero(Transform) # TODO with_base
    if isroot(hlink)
        tf_world_to_hrink = tf_world_to_root
    else
        plink = parent_link(m, hlink)
        tf_plink_to_hrink = zero(Transform) # TODO DUMMY just for testing
        tf_world_to_plink = get_transform_recursive(m, plink)
        tf_world_to_hrink = tf_world_to_plink * tf_plink_to_hrink
    end
    set_cache!(m.tf_cache, hlink.id, tf_world_to_hrink)
    return tf_world_to_hrink
end

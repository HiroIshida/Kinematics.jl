function get_transform(m::Mechanism, link::Link)
    iscached(m, link) && (return get_cache(m, link))
    return _get_transform(m, link)
end

function _get_transform(m::Mechanism, hlink::Link)
    # hlink : here link
    # plink : parent link
    if isroot(hlink)
        tf_world_to_root = zero(Transform) # TODO with_base
        set_cache!(m, hlink, tf_world_to_root)
        return tf_world_to_root
    end
    plink = parent_link(m, hlink)
    hjoint = parent_joint(m, hlink)
    tf_plink_to_hjoint = hjoint.pose
    angle = joint_angle(m, hjoint)

    tf_plink_to_hlink = joint_transform(hjoint, tf_plink_to_hjoint, angle)
    tf_world_to_plink = get_transform(m, plink)
    tf_world_to_hlink = tf_world_to_plink * tf_plink_to_hlink
    set_cache!(m, hlink, tf_world_to_hlink)
    return tf_world_to_hlink
end

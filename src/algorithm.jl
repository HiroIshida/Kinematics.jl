function get_transform(m::Mechanism, link::Link)
    iscached(m, link) && (return get_cache(m, link))
    return _get_transform(m, link)
end

function _get_transform(m::Mechanism, hlink::Link)
    # hlink : here link
    # plink : parent link
    @debugassert isempty(m.tf_stack)
    @debugassert isempty(m.link_id_stack)
    tf_world_to_hlink = zero(Transform) # TODO with_base
    while(!isroot(hlink))
        if iscached(m, hlink) 
            tf_world_to_hlink = get_cache(m, hlink)
            break
        end
        plink = parent_link(m, hlink)
        hjoint = parent_joint(m, hlink)
        tf_plink_to_hjoint = hjoint.pose
        angle = joint_angle(m, hjoint)

        tf_plink_to_hlink = joint_transform(hjoint, tf_plink_to_hjoint, angle)

        push!(m.tf_stack, tf_plink_to_hlink)
        push!(m.link_id_stack, hlink.id)
        hlink = parent_link(m, hlink)
    end

    while(!isempty(m.tf_stack))
        hlink_id = pop!(m.link_id_stack)
        tf_plink_to_hlink = pop!(m.tf_stack)
        tf_world_to_hlink = tf_world_to_hlink * tf_plink_to_hlink
        set_cache!(m, hlink_id, tf_world_to_hlink)
    end
    return tf_world_to_hlink
end

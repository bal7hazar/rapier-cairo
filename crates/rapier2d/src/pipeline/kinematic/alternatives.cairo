//! KD rejected whole-body merge before kinematic dispatch. Kept for gas re-ranking.
use super::super::*;

pub fn user_changes_copied_body(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<ContactPair>,
    dt: Option<Fixed>,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>, Span<(Handle, RigidBody)>, SleepCensus) {
    let mut snapshot = colliders.iter().span();
    let mut dirty = false;
    let mut touched = array![];
    let mut fresh = array![];
    for (handle, collider) in snapshot {
        if !collider.changes.is_empty() {
            collider_changes(*handle, *collider, ref bodies, ref colliders, ref touched, ref fresh);
            dirty = true;
        }
    }
    let mut entries = bodies.iter().span();
    let mut bodies_dirty = false;
    let mut infos = array![];
    let mut census: SleepCensus = Default::default();
    for (handle, body) in entries {
        let mut body = if body.changes.is_empty() {
            *body
        } else {
            bodies_dirty = true;
            body_changes(*handle, *body, ref bodies, ref colliders, ref touched, fresh.span())
        };
        // Mass changes must precede COM-based interpolation. Meter the computing arm
        // so ordinary dynamic/fixed bodies do not pay for trig or a body-set write.
        let mut pending = body.body_type == RigidBodyType::KinematicPositionBased;
        while pending {
            if let Some(dt) = dt {
                body = kinematic::prepare(body, dt);
                let _ = bodies.set(*handle, body);
                bodies_dirty = true;
            }
            pending = false;
        }
        census.count(@body);
        infos
            .append(
                BodyInfo {
                    handle: *handle,
                    body_type: body.body_type,
                    world_com: body.mprops.world_com,
                    dominance: body.dominance.effective_group(body.body_type),
                    sleeping: body.activation.sleeping,
                },
            );
    }
    if !touched.is_empty()
        && !pairs.is_empty()
        && sleeping::wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders) {
        bodies_dirty = true;
        entries = bodies.iter().span();
        let (fresh, recount) = body_infos(entries);
        infos = fresh;
        census = recount;
    } else if bodies_dirty {
        entries = bodies.iter().span();
    }
    if dirty || bodies_dirty {
        snapshot = colliders.iter().span();
    }
    (snapshot, infos.span(), entries, census)
}


pub fn user_changes_per_body(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<ContactPair>,
    dt: Option<Fixed>,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>, Span<(Handle, RigidBody)>, SleepCensus) {
    let mut snapshot = colliders.iter().span();
    let mut dirty = false;
    let mut touched = array![];
    let mut fresh = array![];
    for (handle, collider) in snapshot {
        if !collider.changes.is_empty() {
            collider_changes(*handle, *collider, ref bodies, ref colliders, ref touched, ref fresh);
            dirty = true;
        }
    }
    let mut entries = bodies.iter().span();
    let mut bodies_dirty = false;
    let mut infos = array![];
    let mut census: SleepCensus = Default::default();
    for (handle, body) in entries {
        let (body_type, world_com, dominance, sleeping) = if body.changes.is_empty() {
            census.count(body);
            (
                *body.body_type,
                *body.mprops.world_com,
                body.dominance.effective_group(*body.body_type),
                *body.activation.sleeping,
            )
        } else {
            bodies_dirty = true;
            let body = body_changes(
                *handle, *body, ref bodies, ref colliders, ref touched, fresh.span(),
            );
            census.count(@body);
            (
                body.body_type,
                body.mprops.world_com,
                body.dominance.effective_group(body.body_type),
                body.activation.sleeping,
            )
        };
        // Interpolation changes only velocity, so infos/census remain valid. Read the
        // updated body only in this metered arm, after COM-changing user edits.
        let mut pending = body_type == RigidBodyType::KinematicPositionBased;
        while pending {
            if let Some(dt) = dt {
                kinematic::prepare_in_set(ref bodies, *handle, dt);
                bodies_dirty = true;
            }
            pending = false;
        }
        infos.append(BodyInfo { handle: *handle, body_type, world_com, dominance, sleeping });
    }
    if !touched.is_empty()
        && !pairs.is_empty()
        && sleeping::wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders) {
        bodies_dirty = true;
        entries = bodies.iter().span();
        let (fresh, recount) = body_infos(entries);
        infos = fresh;
        census = recount;
    } else if bodies_dirty {
        entries = bodies.iter().span();
    }
    if dirty || bodies_dirty {
        snapshot = colliders.iter().span();
    }
    (snapshot, infos.span(), entries, census)
}


pub fn user_changes_second_walk(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<ContactPair>,
    dt: Option<Fixed>,
) -> (Span<(Handle, Collider)>, Span<BodyInfo>, Span<(Handle, RigidBody)>, SleepCensus) {
    let mut snapshot = colliders.iter().span();
    let mut dirty = false;
    let mut touched = array![];
    let mut fresh = array![];
    for (handle, collider) in snapshot {
        if !collider.changes.is_empty() {
            collider_changes(*handle, *collider, ref bodies, ref colliders, ref touched, ref fresh);
            dirty = true;
        }
    }
    let mut entries = bodies.iter().span();
    let mut bodies_dirty = false;
    let mut infos = array![];
    let mut census: SleepCensus = Default::default();
    let mut has_kinematic = false;
    for (handle, body) in entries {
        let (body_type, world_com, dominance, sleeping) = if body.changes.is_empty() {
            census.count(body);
            (
                *body.body_type,
                *body.mprops.world_com,
                body.dominance.effective_group(*body.body_type),
                *body.activation.sleeping,
            )
        } else {
            bodies_dirty = true;
            let body = body_changes(
                *handle, *body, ref bodies, ref colliders, ref touched, fresh.span(),
            );
            census.count(@body);
            (
                body.body_type,
                body.mprops.world_com,
                body.dominance.effective_group(body.body_type),
                body.activation.sleeping,
            )
        };
        if body_type == RigidBodyType::KinematicPositionBased {
            has_kinematic = true;
        }
        infos.append(BodyInfo { handle: *handle, body_type, world_com, dominance, sleeping });
    }
    // Only a world with position-based bodies pays for the extra walk. The ordinary
    // body's hot path keeps snapshot field reads and no per-body loop dispatch.
    while has_kinematic {
        if let Some(dt) = dt {
            kinematic::prepare_all(ref bodies, dt);
            bodies_dirty = true;
        }
        has_kinematic = false;
    }
    if !touched.is_empty()
        && !pairs.is_empty()
        && sleeping::wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders) {
        bodies_dirty = true;
        entries = bodies.iter().span();
        let (fresh, recount) = body_infos(entries);
        infos = fresh;
        census = recount;
    } else if bodies_dirty {
        entries = bodies.iter().span();
    }
    if dirty || bodies_dirty {
        snapshot = colliders.iter().span();
    }
    (snapshot, infos.span(), entries, census)
}

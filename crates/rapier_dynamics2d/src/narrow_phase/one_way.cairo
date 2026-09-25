//! Built-in equivalent of upstream `update_as_oneway_platform` (no user hooks).
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_geometry2d::contact::ContactManifold;
use super::PairCollider;

/// Applies the two platforms independently. Each gets a ternary state in `user_data`:
/// unknown=0, allowed=1, forbidden=2; collider 1 occupies the low base-three digit.
/// Angles use each collider's local normal and up, including rotated second colliders.
/// No contact-point impulse is read or changed. Fixed products floor; dot rounds as glam.
pub fn filter(ref manifold: ContactManifold, co1: PairCollider, co2: PairCollider) {
    let (mut state2, mut state1) = DivRem::div_rem(manifold.data.user_data, 3);
    let mut keep = true;
    if let Some(config) = co1.one_way.unbox() {
        let up = config.local_up;
        let (state, accepted) = transition(
            state1, manifold, manifold.local_n1, up, config.cos_allowed_angle,
        );
        state1 = state;
        keep = accepted;
    } else {
        state1 = 0;
    }
    if let Some(config) = co2.one_way.unbox() {
        let up = config.local_up;
        let (state, accepted) = transition(
            state2, manifold, manifold.local_n2, up, config.cos_allowed_angle,
        );
        state2 = state;
        keep = keep && accepted;
    } else {
        state2 = 0;
    }
    manifold.data.user_data = state1 + 3 * state2;
    if !keep {
        manifold.data.num_solver_contacts = 0;
    }
}

fn transition(
    state: u32, manifold: ContactManifold, normal: Vec2, up: Vec2, cosine: Fixed,
) -> (u32, bool) {
    let ok = normal.dot(up) >= cosine;
    if state == 1 {
        (if manifold.data.num_solver_contacts == 0 {
            0
        } else {
            1
        }, true)
    } else if state == 2 {
        let [a, b] = manifold.data.solver_contacts;
        let separated = (manifold.data.num_solver_contacts == 0 || a.dist > ZERO)
            && (manifold.data.num_solver_contacts < 2 || b.dist > ZERO);
        if ok && separated {
            (1, true)
        } else {
            (2, false)
        }
    } else if ok {
        (1, true)
    } else {
        (if manifold.local_n1.length_squared() > (Fixed { raw: 429496730 }) {
            2
        } else {
            0
        }, false)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_geometry2d::contact::{ContactManifold, SolverContact};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use crate::collider::ColliderBuilderTrait;
    use crate::narrow_phase::pair_collider;
    use crate::rigid_body_set::RigidBodySetTrait;
    use super::{filter, transition};

    fn manifold(normal: Vec2, distance: Fixed, count: u8) -> ContactManifold {
        let mut m: ContactManifold = Default::default();
        m.local_n1 = normal;
        m.data.normal = normal;
        m.data.num_solver_contacts = count;
        m
            .data
            .solver_contacts =
                [SolverContact { dist: distance, ..Default::default() }, Default::default()];
        m
    }

    #[test]
    fn test_state_machine() {
        let up = Vec2 { x: ZERO, y: ONE };
        for (state, normal, dist, count, next, keep) in array![
            (0, up, -ONE, 1, 1, true), (0, -up, -ONE, 1, 2, false),
            (0, Vec2 { x: ONE, y: ZERO }, ZERO, 1, 2, false),
            (0, Vec2 { x: ZERO, y: ZERO }, ZERO, 1, 0, false), (2, up, -ONE, 1, 2, false),
            (2, up, ZERO, 1, 2, false), (2, up, HALF, 1, 1, true), (2, -up, HALF, 1, 2, false),
            (1, -up, -ONE, 1, 1, true), (1, up, ZERO, 0, 0, true),
        ] {
            assert_eq!(
                transition(state, manifold(normal, dist, count), normal, up, ONE), (next, keep),
            );
        }
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_transition() {
        let up = Vec2 { x: ZERO, y: opaque(ONE) };
        let _ = opaque(transition(opaque(2), manifold(up, HALF, 1), up, up, ONE));
    }
    fn inputs(
        second: bool, rotated: bool, normal: Vec2,
    ) -> (ContactManifold, super::PairCollider, super::PairCollider) {
        let mut bodies = RigidBodySetTrait::new();
        let rotation = if rotated {
            Rot2 { re: ZERO, im: ONE }
        } else {
            Rot2 { re: ONE, im: ZERO }
        };
        let platform = ColliderBuilderTrait::cuboid(ONE, HALF)
            .rotation(rotation)
            .one_way(Vec2 { x: ZERO, y: ONE }, ZERO)
            .build();
        let ball = ColliderBuilderTrait::ball(HALF).build();
        let (a, b) = if second {
            (ball, platform)
        } else {
            (platform, ball)
        };
        let co1 = pair_collider(Handle { index: 0, generation: 0 }, a, ref bodies);
        let co2 = pair_collider(Handle { index: 1, generation: 0 }, b, ref bodies);
        let mut m = manifold(if second {
            -normal
        } else {
            normal
        }, -HALF, 1);
        m.local_n1 = co1.pose.rotation.inverse_rotate(m.data.normal);
        m.local_n2 = -co2.pose.rotation.inverse_rotate(m.data.normal);
        (m, co1, co2)
    }
    fn filter_case(second: bool, rotated: bool, normal: Vec2) -> ContactManifold {
        let (mut m, a, b) = inputs(second, rotated, normal);
        let mut other = m;
        filter(ref m, a, b);
        super::alternatives::filter(ref other, a, b);
        assert_eq!(m, other);
        m
    }
    #[test]
    fn test_platform_frames_and_pair_order() {
        for second in array![false, true] {
            for (rotated, good, bad) in array![
                (false, Vec2 { x: ZERO, y: ONE }, Vec2 { x: ONE, y: ZERO }),
                (true, Vec2 { x: -ONE, y: ZERO }, Vec2 { x: ZERO, y: ONE }),
            ] {
                assert_eq!(filter_case(second, rotated, good).data.num_solver_contacts, 1);
                assert_eq!(filter_case(second, rotated, -good).data.num_solver_contacts, 0);
                assert_eq!(filter_case(second, rotated, bad).data.num_solver_contacts, 0);
            }
        }
    }
    #[test]
    fn gas_filter() {
        let (mut m, a, b) = inputs(opaque(true), opaque(true), Vec2 { x: -ONE, y: ZERO });
        filter(ref m, a, b);
        let _ = opaque(m);
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260925)]
    fn fuzz_local_world_equivalent(second: bool, rotated: bool, state: u32, separating: bool) {
        let (_, state) = DivRem::div_rem(state, 9);
        let (mut a, co1, co2) = inputs(second, rotated, Vec2 { x: -ONE, y: ZERO });
        a.data.user_data = state;
        if separating {
            let [mut p, q] = a.data.solver_contacts;
            p.dist = HALF;
            a.data.solver_contacts = [p, q];
        }
        let mut b = a;
        filter(ref a, co1, co2);
        super::alternatives::filter(ref b, co1, co2);
        assert_eq!(a, b);
    }

    #[test]
    fn gas_filter_world() {
        let (mut m, a, b) = inputs(opaque(true), opaque(true), Vec2 { x: -ONE, y: ZERO });
        super::alternatives::filter(ref m, a, b);
        let _ = opaque(m);
    }
}

#[cfg(test)]
mod alternatives {
    use rapier_math::rot2::Rot2Trait;
    use super::*;
    pub fn filter(ref manifold: ContactManifold, co1: PairCollider, co2: PairCollider) {
        let (mut state2, mut state1) = DivRem::div_rem(manifold.data.user_data, 3);
        let mut keep = true;
        if let Some(config) = co1.one_way.unbox() {
            let up = co1.pose.rotation.rotate(config.local_up);
            let (state, accepted) = transition(
                state1, manifold, manifold.data.normal, up, config.cos_allowed_angle,
            );
            state1 = state;
            keep = accepted;
        } else {
            state1 = 0;
        }
        if let Some(config) = co2.one_way.unbox() {
            let up = -co2.pose.rotation.rotate(config.local_up);
            let (state, accepted) = transition(
                state2, manifold, manifold.data.normal, up, config.cos_allowed_angle,
            );
            state2 = state;
            keep = keep && accepted;
        } else {
            state2 = 0;
        }
        manifold.data.user_data = state1 + 3 * state2;
        if !keep {
            manifold.data.num_solver_contacts = 0;
        }
    }
}

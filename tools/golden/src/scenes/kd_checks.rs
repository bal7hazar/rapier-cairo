//! KD prerequisite: match the Cairo world_step pusher regressions against pinned upstream.
use super::*;

#[test]
fn kd_upstream_pusher_semantics() {
    for (kind, dominance) in [
        (RigidBodyType::Dynamic, 0),
        (RigidBodyType::KinematicVelocityBased, 0),
        (RigidBodyType::Dynamic, 1),
    ] {
        let params = IntegrationParameters {
            dt: Q(71582788).f(),
            contact_recycling: false,
            contact_clustering: false,
            max_ccd_substeps: 0,
            ..Default::default()
        };
        let mut pipeline = PhysicsPipeline::new();
        let mut islands = IslandManager::new();
        let mut broad_phase = DefaultBroadPhase::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = ImpulseJointSet::new();
        let mut multibodies = MultibodyJointSet::new();
        let mut ccd = CCDSolver::new();
        let left = bodies.insert(
            RigidBodyBuilder::new(kind)
                .translation(Vector::new(-1.0, 0.0))
                .linvel(Vector::new(1.0, 0.0))
                .dominance_group(dominance)
                .can_sleep(false),
        );
        let right = bodies.insert(RigidBodyBuilder::dynamic().can_sleep(false));
        for handle in [left, right] {
            colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.5, 0.5).friction(0.0),
                handle,
                &mut bodies,
            );
        }
        pipeline.step(
            Vector::ZERO,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut joints,
            &mut multibodies,
            &mut ccd,
            &(),
            &(),
        );
        let driver = bodies[left].linvel().x;
        let passenger = bodies[right].linvel().x;
        println!("{kind:?} dominance={dominance}: driver={driver}, passenger={passenger}");
        if dominance == 0 {
            assert!(passenger > 0.0);
        } else {
            // Upstream treats the dominance-superior endpoint as world-attached.
            assert_eq!(passenger, 0.0);
        }
        if kind == RigidBodyType::KinematicVelocityBased || dominance != 0 {
            assert_eq!(driver, 1.0);
        } else {
            assert!(driver < 1.0);
        }
    }
}

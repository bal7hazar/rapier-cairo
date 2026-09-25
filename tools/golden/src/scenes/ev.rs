//! EV: the upstream one-way helper and contact-force event handler.
use super::*;
use std::sync::Mutex;

pub(super) fn scenes() -> Vec<SceneSpec> {
    vec![
        SceneSpec {
            id: "one_way_jump", note: "ball launches upward through a one-way slab and lands; upstream update_as_oneway_platform(+Y, 0.1)",
            bodies: vec![ground(0.0, 0.0), dynamic("ball", QPose::translation(0.0, -2.0), collider(ShapeSpec::ball(0.25), 0.0, 0.0))],
            joints: vec![], can_sleep: false,
        },
        SceneSpec {
            id: "force_event_drop", note: "unit box dropped on a solid slab with CONTACT_FORCE_EVENTS and threshold 20 N; normal impulses only",
            bodies: vec![ground(0.0, 0.0), dynamic("box", QPose::translation(0.0, 2.0), collider(ShapeSpec::cuboid(0.5, 0.5), 0.0, 0.0))],
            joints: vec![], can_sleep: false,
        },
    ]
}

pub(super) struct Hook {
    pub platform: Option<ColliderHandle>,
}
impl PhysicsHooks for Hook {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        if let Some(platform) = self.platform {
            let up = if context.collider1 == platform {
                Vector::Y
            } else {
                -Vector::Y
            };
            context.update_as_oneway_platform(up, Q::snap(0.1).f());
        }
    }
}

#[derive(Default)]
pub(super) struct Collector(pub Mutex<Vec<ContactForceEvent>>);
impl EventHandler for Collector {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
    }
    fn handle_contact_force_event(
        &self,
        dt: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        pair: &ContactPair,
        magnitude: Real,
    ) {
        self.0
            .lock()
            .unwrap()
            .push(ContactForceEvent::from_contact_pair(dt, pair, magnitude));
    }
}

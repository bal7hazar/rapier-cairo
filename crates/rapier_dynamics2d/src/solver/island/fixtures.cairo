//! Analytic contacts used only for driver regressions and benchmarks.
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::{ContactManifold, NEW_CONTACT_BIT, SolverContact, SolverFlags};
use rapier_math::pose2::Pose2;
use crate::rigid_body::RigidBodyVelocity;
use super::super::body::SolverBody;
use super::super::body_store::BodyStep;
pub(crate) fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
pub(crate) fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}
pub(crate) fn stack(n: u32) -> (Array<SolverBody>, Array<BodyStep>, Array<ContactManifold>) {
    let p: IntegrationParameters = Default::default();
    let mut bs = array![];
    let mut steps = array![];
    let mut ms = array![];
    let mut i = 0;
    while i != n {
        let b = SolverBody {
            handle: h(i),
            position: Pose2 {
                translation: v(ZERO, HALF + FixedTrait::from_int(i.try_into().unwrap())),
                ..Default::default(),
            },
            im: v(ONE, ONE),
            ii: FixedTrait::from_int(6),
            ..Default::default(),
        };
        let mut m: ContactManifold = Default::default();
        m.num_points = 2;
        m.data.num_solver_contacts = 2;
        m.data.normal = v(ZERO, ONE);
        m.data.friction = HALF;
        m.data.rigid_body1 = if i == 0 {
            None
        } else {
            Some(h(i - 1))
        };
        m.data.rigid_body2 = Some(h(i));
        m.data.solver_flags = SolverFlags { bits: 1 };
        let a = SolverContact {
            anchor1: v(HALF, if i == 0 {
                ZERO
            } else {
                HALF
            }),
            anchor2: v(HALF, -HALF),
            contact_id: NEW_CONTACT_BIT,
            ..Default::default(),
        };
        let c = SolverContact {
            anchor1: v(-HALF, a.anchor1.y),
            anchor2: v(-HALF, -HALF),
            contact_id: NEW_CONTACT_BIT + 1,
            ..a,
        };
        m.data.solver_contacts = [a, c];
        bs.append(b);
        steps
            .append(
                BodyStep {
                    increment: RigidBodyVelocity {
                        linvel: v(ZERO, Fixed { raw: -42133629174 } * p.substep_dt()), angvel: ZERO,
                    },
                    damping: Default::default(),
                    local_com: Default::default(),
                    moving: true,
                },
            );
        ms.append(m);
        i += 1;
    }
    (bs, steps, ms)
}

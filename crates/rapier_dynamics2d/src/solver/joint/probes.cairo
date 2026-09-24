//! Public-builder chains shared by OJ stage probes and equivalence tests.
mod chains;
mod construction;
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_math::pose2::Pose2;
use rapier_testing::opaque;
use crate::joint::{FixedJointBuilderTrait, PrismaticJointBuilderTrait, RevoluteJointBuilderTrait};
use crate::solver::body::SolverBody;
use crate::solver::body_store::BodyStep;
use super::*;

pub(crate) fn chain(n: u32, kind: u8) -> (Array<SolverBody>, Array<ImpulseJoint>, Array<BodyStep>) {
    let m = MassPropertiesTrait::from_ball(ONE, HALF);
    let mut bs = array![
        SolverBody { handle: Handle { index: 0, generation: 1 }, ..Default::default() },
    ];
    let mut js = array![];
    let mut steps = array![
        BodyStep {
            increment: Default::default(),
            local_com: Default::default(),
            damping: Default::default(),
            moving: false,
        },
    ];
    let mut i = 0;
    while i != n {
        let first = Handle { index: i, generation: 1 };
        let second = Handle { index: i + 1, generation: 1 };
        let a = Vec2 { x: ONE, y: ZERO };
        let b = Vec2 { x: -ONE, y: ZERO };
        let data = match kind {
            0 => RevoluteJointBuilderTrait::new().local_anchor1(a).local_anchor2(b).build(),
            1 => PrismaticJointBuilderTrait::new(a).local_anchor1(a).local_anchor2(b).build(),
            _ => FixedJointBuilderTrait::new().local_anchor1(a).local_anchor2(b).build(),
        };
        js.append(ImpulseJoint { body1: first, body2: second, data, impulses: [ZERO, ZERO, ZERO] });
        let x = FixedTrait::from_int((i + 1).try_into().unwrap()) * FixedTrait::from_int(2);
        bs
            .append(
                SolverBody {
                    handle: second,
                    position: Pose2 { translation: Vec2 { x, y: ZERO }, ..Default::default() },
                    linvel: Vec2 { x: ZERO, y: Fixed { raw: -175556788 } },
                    im: Vec2 { x: m.inv_mass, y: m.inv_mass },
                    ii: m.inv_principal_inertia,
                    ..Default::default(),
                },
            );
        steps
            .append(
                BodyStep {
                    increment: Default::default(),
                    local_com: Default::default(),
                    damping: Default::default(),
                    moving: true,
                },
            );
        i += 1;
    }
    (opaque(bs), opaque(js), opaque(steps))
}

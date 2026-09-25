//! Public-builder chains shared by OJ stage probes and equivalence tests.
mod chains;
mod construction;
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_math::pose2::Pose2;
use rapier_testing::opaque;
use crate::joint::{
    FixedJointBuilderTrait, GenericJoint, MotorModel, PrismaticJointBuilderTrait,
    RevoluteJointBuilderTrait,
};
use crate::solver::body::SolverBody;
use crate::solver::body_store::BodyStep;
use super::*;

/// Chain joint kinds: 0 revolute, 1 prismatic, 2 fixed; controlled revolute 3 interior limit,
/// 4 violated limit, 5 velocity motor, 6 position motor (force based); 7 violated prismatic limit.
pub(crate) fn joint_data(kind: u8, a: Vec2, b: Vec2) -> GenericJoint {
    let revolute = RevoluteJointBuilderTrait::new().local_anchor1(a).local_anchor2(b);
    let prismatic = PrismaticJointBuilderTrait::new(a).local_anchor1(a).local_anchor2(b);
    match kind {
        0 => revolute.build(),
        1 => prismatic.build(),
        2 => FixedJointBuilderTrait::new().local_anchor1(a).local_anchor2(b).build(),
        3 => revolute.limits([-ONE, ONE]).build(),
        4 => revolute.limits([HALF, ONE]).build(),
        5 => revolute.motor_velocity(ONE, ONE).build(),
        6 => revolute
            .motor_position(HALF, ONE, Fixed { raw: 429496730 })
            .motor_model(MotorModel::ForceBased)
            .build(),
        _ => prismatic.limits([HALF, ONE]).build(),
    }
}
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
        let data = joint_data(kind, a, b);
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

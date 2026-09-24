//! Configuration parity of generic, revolute and prismatic value builders.
use fixed::{HALF, ONE, ZERO};
use glam::Vec2;
use rapier_testing::opaque;
use super::*;

#[test]
fn test_builder_controls_match_generic_setters() {
    for axis in [0_u8, 2].span() {
        let axis = *axis;
        let locks = if axis == 0 {
            LOCKED_PRISMATIC_AXES
        } else {
            LOCKED_REVOLUTE_AXES
        };
        let base = GenericJointBuilderTrait::new(locks);
        let mut expected = base.build();
        expected.set_limits(axis, [-HALF, ONE]);
        expected.set_motor_model(axis, MotorModel::ForceBased);
        expected.set_motor_max_force(axis, HALF);
        expected.set_motor(axis, HALF, ONE, ONE, HALF);
        let g = base
            .limits(axis, [-HALF, ONE])
            .motor_model(axis, MotorModel::ForceBased)
            .motor_max_force(axis, HALF)
            .set_motor(axis, HALF, ONE, ONE, HALF)
            .build();
        let specialized = if axis == 0 {
            PrismaticJointBuilderTrait::new(Vec2 { x: ONE, y: ZERO })
                .limits([-HALF, ONE])
                .motor_model(MotorModel::ForceBased)
                .motor_max_force(HALF)
                .set_motor(HALF, ONE, ONE, HALF)
                .build()
        } else {
            RevoluteJointBuilderTrait::new()
                .limits([-HALF, ONE])
                .motor_model(MotorModel::ForceBased)
                .motor_max_force(HALF)
                .motor(HALF, ONE, ONE, HALF)
                .build()
        };
        assert_eq!(g, expected);
        assert_eq!(specialized, expected);
        expected.set_motor_velocity(axis, -ONE, ONE);
        let g = GenericJointBuilder { data: g }.motor_velocity(axis, -ONE, ONE).build();
        let specialized = if axis == 0 {
            PrismaticJointBuilder { data: specialized }.motor_velocity(-ONE, ONE).build()
        } else {
            RevoluteJointBuilder { data: specialized }.motor_velocity(-ONE, ONE).build()
        };
        assert_eq!(g, expected);
        assert_eq!(specialized, expected);
        expected.set_motor_position(axis, -HALF, HALF, ONE);
        let g = GenericJointBuilder { data: g }.motor_position(axis, -HALF, HALF, ONE).build();
        let specialized = if axis == 0 {
            PrismaticJointBuilder { data: specialized }.motor_position(-HALF, HALF, ONE).build()
        } else {
            RevoluteJointBuilder { data: specialized }.motor_position(-HALF, HALF, ONE).build()
        };
        assert_eq!(g, expected);
        assert_eq!(specialized, expected);
    }
}
#[test]
fn test_builders_do_not_enable_model_or_force_only() {
    let g = GenericJointBuilderTrait::new(LOCKED_REVOLUTE_AXES)
        .motor_model(2, MotorModel::ForceBased)
        .motor_max_force(2, ZERO)
        .build();
    let r = RevoluteJointBuilderTrait::new()
        .motor_model(MotorModel::ForceBased)
        .motor_max_force(ZERO)
        .build();
    let p = PrismaticJointBuilderTrait::new(Vec2 { x: ONE, y: ZERO })
        .motor_model(MotorModel::ForceBased)
        .motor_max_force(ZERO)
        .build();
    assert_eq!(g, r);
    assert_eq!(g.motor_axes.bits, 0);
    assert_eq!(p.motor_axes.bits, 0);
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
fn gas_generic_limits() {
    let _ = opaque(
        opaque(GenericJointBuilderTrait::new(opaque(LOCKED_REVOLUTE_AXES)))
            .limits(opaque(2), opaque([-HALF, ONE])),
    );
}

#[test]
fn gas_generic_motor_model() {
    let _ = opaque(
        opaque(GenericJointBuilderTrait::new(opaque(LOCKED_REVOLUTE_AXES)))
            .motor_model(opaque(2), opaque(MotorModel::ForceBased)),
    );
}

#[test]
fn gas_generic_motor_max_force() {
    let _ = opaque(
        opaque(GenericJointBuilderTrait::new(opaque(LOCKED_REVOLUTE_AXES)))
            .motor_max_force(opaque(2), opaque(ONE)),
    );
}

#[test]
fn gas_generic_motor_velocity() {
    let _ = opaque(
        opaque(GenericJointBuilderTrait::new(opaque(LOCKED_REVOLUTE_AXES)))
            .motor_velocity(opaque(2), opaque(ONE), opaque(HALF)),
    );
}

#[test]
fn gas_generic_motor_position() {
    let _ = opaque(
        opaque(GenericJointBuilderTrait::new(opaque(LOCKED_REVOLUTE_AXES)))
            .motor_position(opaque(2), opaque(HALF), opaque(ONE), opaque(ONE)),
    );
}

#[test]
fn gas_generic_set_motor() {
    let _ = opaque(
        opaque(GenericJointBuilderTrait::new(opaque(LOCKED_REVOLUTE_AXES)))
            .set_motor(opaque(2), opaque(HALF), opaque(ONE), opaque(ONE), opaque(HALF)),
    );
}

#[test]
fn gas_revolute_limits() {
    let _ = opaque(opaque(RevoluteJointBuilderTrait::new()).limits(opaque([-HALF, ONE])));
}

#[test]
fn gas_revolute_motor_model() {
    let _ = opaque(
        opaque(RevoluteJointBuilderTrait::new()).motor_model(opaque(MotorModel::ForceBased)),
    );
}

#[test]
fn gas_revolute_motor_max_force() {
    let _ = opaque(opaque(RevoluteJointBuilderTrait::new()).motor_max_force(opaque(ONE)));
}

#[test]
fn gas_revolute_motor_velocity() {
    let _ = opaque(
        opaque(RevoluteJointBuilderTrait::new()).motor_velocity(opaque(ONE), opaque(HALF)),
    );
}

#[test]
fn gas_revolute_motor_position() {
    let _ = opaque(
        opaque(RevoluteJointBuilderTrait::new())
            .motor_position(opaque(HALF), opaque(ONE), opaque(ONE)),
    );
}

#[test]
fn gas_revolute_motor() {
    let _ = opaque(
        opaque(RevoluteJointBuilderTrait::new())
            .motor(opaque(HALF), opaque(ONE), opaque(ONE), opaque(HALF)),
    );
}

#[test]
fn gas_prismatic_limits() {
    let _ = opaque(
        opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
            .limits(opaque([-HALF, ONE])),
    );
}

#[test]
fn gas_prismatic_motor_model() {
    let _ = opaque(
        opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
            .motor_model(opaque(MotorModel::ForceBased)),
    );
}

#[test]
fn gas_prismatic_motor_max_force() {
    let _ = opaque(
        opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
            .motor_max_force(opaque(ONE)),
    );
}

#[test]
fn gas_prismatic_motor_velocity() {
    let _ = opaque(
        opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
            .motor_velocity(opaque(ONE), opaque(HALF)),
    );
}

#[test]
fn gas_prismatic_motor_position() {
    let _ = opaque(
        opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
            .motor_position(opaque(HALF), opaque(ONE), opaque(ONE)),
    );
}

#[test]
fn gas_prismatic_set_motor() {
    let _ = opaque(
        opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
            .set_motor(opaque(HALF), opaque(ONE), opaque(ONE), opaque(HALF)),
    );
}

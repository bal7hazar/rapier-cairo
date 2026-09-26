//! Typed joint views (`FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `PinSlotJoint`, `RopeJoint`,
//! `SpringJoint`), the `GenericJoint` accessors and the `ImpulseJointSet` queries (JA1).
use fixed::{Fixed, HALF, MAX, MIN, ONE, ZERO};
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use super::*;

const NEG_HALF: Fixed = Fixed { raw: -2147483648 };
const X_AXIS: Vec2 = Vec2 { x: ONE, y: ZERO };
const Y_AXIS: Vec2 = Vec2 { x: ZERO, y: ONE };

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn body(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

fn soft() -> SpringCoefficients {
    SpringCoefficients { natural_frequency: ONE, damping_ratio: HALF }
}

/// Every typed view wraps exactly the joint its builder (or `new`) makes, converts back to it and
/// is recognised by `as_*`.
#[test]
fn test_typed_views_round_trip_to_generic_joint() {
    let (a1, a2) = (v(HALF, ONE), v(NEG_HALF, ZERO));
    let fixed = FixedJointBuilderTrait::new().local_anchor1(a1).local_anchor2(a2).build();
    let view = FixedJoint { data: fixed };
    assert_eq!(view.into(), fixed);
    assert_eq!(FixedJointTrait::new().data, FixedJointBuilderTrait::new().build());
    assert_eq!(Default::<FixedJoint>::default(), FixedJointTrait::new());
    assert_eq!(fixed.as_fixed(), Some(view));
    assert!(fixed.as_revolute().is_none());
    assert!(fixed.as_rope().is_none());
    let revolute = RevoluteJointBuilderTrait::new().local_anchor1(a1).build();
    assert_eq!(RevoluteJointTrait::new().data, RevoluteJointBuilderTrait::new().build());
    assert_eq!(revolute.as_revolute(), Some(RevoluteJoint { data: revolute }));
    assert_eq!(RevoluteJoint { data: revolute }.into(), revolute);
    assert!(revolute.as_prismatic().is_none());
    let prismatic = PrismaticJointBuilderTrait::new(Y_AXIS).local_anchor2(a2).build();
    assert_eq!(
        PrismaticJointTrait::new(Y_AXIS).data, PrismaticJointBuilderTrait::new(Y_AXIS).build(),
    );
    assert_eq!(prismatic.as_prismatic(), Some(PrismaticJoint { data: prismatic }));
    assert_eq!(PrismaticJoint { data: prismatic }.into(), prismatic);
    let rope = RopeJointBuilderTrait::new(ONE).local_anchor1(a1).build();
    assert_eq!(RopeJointTrait::new(ONE).data, RopeJointBuilderTrait::new(ONE).build());
    assert_eq!(rope.as_rope(), Some(RopeJoint { data: rope }));
    assert_eq!(RopeJoint { data: rope }.into(), rope);
    let spring = SpringJointBuilderTrait::new(ONE, HALF, HALF).build();
    assert_eq!(SpringJointTrait::new(ONE, HALF, HALF).data, spring);
    assert_eq!(SpringJoint { data: spring }.into(), spring);
    let slot = PinSlotJointBuilderTrait::new(X_AXIS).build();
    assert_eq!(PinSlotJointTrait::new(X_AXIS).data, slot);
    assert_eq!(PinSlotJoint { data: slot }.into(), slot);
    assert_eq!(slot.locked_axes, LOCKED_PIN_SLOT_AXES);
    // The builders convert to the generic joint as well.
    assert_eq!(Into::<_, GenericJoint>::into(FixedJointBuilder { data: fixed }), fixed);
    assert_eq!(Into::<_, GenericJoint>::into(RevoluteJointBuilder { data: revolute }), revolute);
    assert_eq!(Into::<_, GenericJoint>::into(PrismaticJointBuilder { data: prismatic }), prismatic);
    assert_eq!(Into::<_, GenericJoint>::into(RopeJointBuilder { data: rope }), rope);
    assert_eq!(Into::<_, GenericJoint>::into(SpringJointBuilder { data: spring }), spring);
    assert_eq!(Into::<_, GenericJoint>::into(PinSlotJointBuilder { data: slot }), slot);
    assert_eq!(
        Into::<_, GenericJoint>::into(GenericJointBuilderTrait::new(LIN_X)),
        GenericJointTrait::new(LIN_X),
    );
}

/// The typed setters and getters read and write the same `GenericJoint` fields as the generic ones.
#[test]
fn test_typed_accessors_match_generic_joint() {
    let (a1, a2) = (v(HALF, ONE), v(NEG_HALF, ZERO));
    let frame = Pose2 { translation: a1, rotation: Rot2 { re: ZERO, im: ONE } };
    // Fixed: frames, anchors, softness, contacts.
    let mut f = FixedJointTrait::new();
    f.set_local_frame1(frame);
    f.set_local_frame2(frame);
    f.set_local_anchor2(a2);
    f.set_softness(soft());
    f.set_contacts_enabled(false);
    let mut g: GenericJoint = FixedJointTrait::new().into();
    g.set_local_frame1(frame);
    g.set_local_frame2(frame);
    g.set_local_anchor2(a2);
    g.set_softness(soft());
    g.set_contacts_enabled(false);
    assert_eq!(f.data(), g);
    assert_eq!((f.local_frame1(), f.local_frame2()), (frame, Pose2 { translation: a2, ..frame }));
    assert_eq!((f.local_anchor1(), f.local_anchor2()), (a1, a2));
    assert_eq!(f.softness(), soft());
    assert!(!f.contacts_enabled());
    // Revolute: angular limits and motor, `None` until enabled.
    let mut r = RevoluteJointTrait::new();
    assert!(r.limits().is_none() && r.motor().is_none());
    r.set_limits([NEG_HALF, HALF]);
    r.set_motor_model(MotorModel::ForceBased);
    r.set_motor_max_force(ONE);
    assert!(r.motor().is_none());
    r.set_motor(HALF, ONE, ONE, HALF);
    let mut g: GenericJoint = RevoluteJointTrait::new().into();
    g.set_limits(2, [NEG_HALF, HALF]);
    g.set_motor_model(2, MotorModel::ForceBased);
    g.set_motor_max_force(2, ONE);
    g.set_motor(2, HALF, ONE, ONE, HALF);
    assert_eq!(r.data(), g);
    assert_eq!(r.limits(), g.limits(2));
    assert_eq!(r.motor(), g.motor(2));
    r.set_motor_velocity(-ONE, HALF);
    r.set_motor_position(ONE, HALF, ONE);
    g.set_motor_velocity(2, -ONE, HALF);
    g.set_motor_position(2, ONE, HALF, ONE);
    assert_eq!(r.data(), g);
    // Prismatic and pin-slot: the linear axis 0, the frame axes.
    let mut p = PrismaticJointTrait::new(X_AXIS);
    p.set_limits([ZERO, ONE]);
    p.set_motor_velocity(ONE, HALF);
    p.set_local_axis1(Y_AXIS);
    p.set_local_anchor1(a1);
    let mut s = PinSlotJointTrait::new(X_AXIS);
    s.set_limits([ZERO, ONE]);
    s.set_motor_velocity(ONE, HALF);
    s.set_local_axis1(Y_AXIS);
    s.set_local_anchor1(a1);
    assert_eq!(p.limits(), Some(JointLimits { min: ZERO, max: ONE, impulse: ZERO }));
    assert_eq!(s.limits(), p.limits());
    assert_eq!(s.motor(), p.motor());
    assert_eq!(p.motor().unwrap().stiffness, ZERO);
    assert_eq!(p.local_axis2(), X_AXIS);
    assert_eq!(
        p.local_axis1(), v(a1.x, ONE + a1.y),
    ); // upstream: the frame applied to X as a point.
    assert_eq!(p.data().locked_axes, LOCKED_PRISMATIC_AXES);
    assert_eq!(s.data().locked_axes, LOCKED_PIN_SLOT_AXES);
    // Rope and spring: the coupled axis 0.
    let mut rope = RopeJointTrait::new(ONE);
    assert_eq!(rope.max_distance(), ONE);
    rope.set_max_distance(HALF);
    rope.set_motor_position(HALF, ONE, ONE);
    rope.set_local_anchor1(a1);
    rope.set_softness(soft());
    assert_eq!(rope.max_distance(), HALF);
    assert_eq!(rope.motor(0).unwrap().target_pos, HALF);
    assert!(rope.motor(1).is_none());
    assert_eq!((rope.local_anchor1(), rope.softness()), (a1, soft()));
    assert_eq!(rope.data().limits(0), Some(JointLimits { min: ZERO, max: HALF, impulse: ZERO }));
    let mut bare = rope.data();
    bare.limit_axes = Default::default();
    assert_eq!(RopeJoint { data: bare }.max_distance(), MAX);
    let mut spring = SpringJointTrait::new(ONE, HALF, HALF);
    assert_eq!(spring.data().motor_model(0), Some(MotorModel::ForceBased));
    spring.set_spring_model(MotorModel::AccelerationBased);
    spring.set_local_anchor2(a2);
    spring.set_contacts_enabled(false);
    assert_eq!(spring.data().motor_model(0), Some(MotorModel::AccelerationBased));
    assert_eq!((spring.local_anchor1(), spring.local_anchor2()), (origin(), a2));
    assert!(!spring.contacts_enabled());
}

fn origin() -> Vec2 {
    Vec2 { x: ZERO, y: ZERO }
}

/// `RevoluteJoint::angle` is the relative rotation of the two frames in world space.
#[test]
fn test_revolute_angle() {
    let quarter = Rot2 { re: ZERO, im: ONE };
    let half_turn = Rot2 { re: -ONE, im: ZERO };
    let identity = Rot2 { re: ONE, im: ZERO };
    let mut j = RevoluteJointTrait::new();
    let cases = array![
        (identity, identity, ZERO), (identity, quarter, fixed::FRAC_PI_2),
        (quarter, identity, -fixed::FRAC_PI_2), (quarter, half_turn, fixed::FRAC_PI_2),
    ];
    for (r1, r2, expected) in cases {
        let angle = j.angle(r1, r2);
        let d = if angle > expected {
            angle - expected
        } else {
            expected - angle
        };
        assert!(d.raw < 16, "angle");
    }
    // A frame rotation offsets the angle.
    j.data.local_frame2.rotation = quarter;
    assert!((j.angle(identity, identity) - fixed::FRAC_PI_2).raw < 16);
}

/// `GenericJoint` construction and accessors.
#[test]
fn test_generic_joint_accessors() {
    let j = GenericJointTrait::new(LOCKED_PRISMATIC_AXES);
    assert_eq!(j, GenericJoint { locked_axes: LOCKED_PRISMATIC_AXES, ..Default::default() });
    let mut j = GenericJointTrait::new(LIN_X);
    j.lock_axes(ANG_X);
    assert_eq!(j.locked_axes.bits, 5);
    j.lock_axes(LIN_X);
    assert_eq!(j.locked_axes.bits, 5);
    // set_enabled: (initial state, argument) -> state, as upstream.
    let cases = array![
        (JointEnabled::Enabled, true, JointEnabled::Enabled),
        (JointEnabled::Enabled, false, JointEnabled::Disabled),
        (JointEnabled::Disabled, true, JointEnabled::Enabled),
        (JointEnabled::Disabled, false, JointEnabled::Disabled),
        (JointEnabled::DisabledByAttachedBody, true, JointEnabled::DisabledByAttachedBody),
        (JointEnabled::DisabledByAttachedBody, false, JointEnabled::Disabled),
    ];
    for (state, arg, expected) in cases {
        let mut j = GenericJoint { enabled: state, ..Default::default() };
        assert_eq!(j.is_enabled(), state == JointEnabled::Enabled);
        j.set_enabled(arg);
        assert_eq!(j.enabled, expected);
        assert_eq!(j.is_enabled(), expected == JointEnabled::Enabled);
    }
    // Frames, anchors and axes.
    let mut j: GenericJoint = Default::default();
    j.set_local_anchor1(v(HALF, ONE));
    j.set_local_anchor2(v(ONE, HALF));
    assert_eq!((j.local_anchor1(), j.local_anchor2()), (v(HALF, ONE), v(ONE, HALF)));
    j.set_local_axis2(Y_AXIS);
    assert_eq!(j.local_frame2.rotation, Rot2 { re: ZERO, im: ONE });
    assert_eq!(j.local_axis2(), v(ONE, ONE + HALF));
    assert_eq!(GenericJointTrait::complete_ang_frame(X_AXIS), Rot2 { re: ONE, im: ZERO });
    j.set_local_frame1(Pose2 { translation: origin(), rotation: Rot2 { re: ZERO, im: -ONE } });
    assert_eq!(j.local_axis1(), v(ZERO, -ONE));
    assert!(j.contacts_enabled());
    j.set_contacts_enabled(false);
    assert!(!j.contacts_enabled());
    j.set_softness(soft());
    assert_eq!(j.softness, soft());
    // limits / motor / motor_model are `None` until enabled.
    let mut j: GenericJoint = Default::default();
    for axis in [0_u8, 1, 2].span() {
        assert!(j.limits(*axis).is_none() && j.motor(*axis).is_none());
        assert!(j.motor_model(*axis).is_none());
    }
    j.set_limits(1, [NEG_HALF, HALF]);
    j.set_motor_model(1, MotorModel::ForceBased);
    assert!(j.motor_model(1).is_none());
    j.set_motor_velocity(1, ONE, HALF);
    assert_eq!(j.limits(1), Some(JointLimits { min: NEG_HALF, max: HALF, impulse: ZERO }));
    assert_eq!(j.motor_model(1), Some(MotorModel::ForceBased));
    assert_eq!(j.motor(1).unwrap().target_vel, ONE);
    assert!(j.limits(0).is_none() && j.motor(2).is_none());
}

/// `flip` swaps the frames, mirrors the uncoupled limits and negates the motor targets; it is an
/// involution and keeps the unbounded limits unbounded.
#[test]
fn test_flip() {
    let (f1, f2) = (
        Pose2 { translation: v(HALF, ONE), rotation: Rot2 { re: ONE, im: ZERO } },
        Pose2 { translation: v(ONE, HALF), rotation: Rot2 { re: ZERO, im: ONE } },
    );
    let mut j = GenericJoint { local_frame1: f1, local_frame2: f2, ..Default::default() };
    j.set_limits(0, [NEG_HALF, ONE]);
    j.set_limits(2, [ZERO, HALF]);
    j.set_motor_position(1, HALF, ONE, ONE);
    j.set_motor_velocity(2, ONE, HALF);
    j.coupled_axes = ANG_X;
    let original = j;
    j.flip();
    assert_eq!((j.local_frame1, j.local_frame2), (f2, f1));
    let [x, y, w] = j.limits;
    assert_eq!((x.min, x.max), (-ONE, HALF));
    assert_eq!((y.min, y.max), (MIN, MAX)); // unbounded stays unbounded
    assert_eq!((w.min, w.max), (ZERO, HALF)); // coupled: untouched
    let [_, my, mw] = j.motors;
    assert_eq!((my.target_pos, my.target_vel), (NEG_HALF, ZERO));
    assert_eq!((mw.target_pos, mw.target_vel), (ZERO, -ONE));
    j.flip();
    assert_eq!(j, original);
}

/// `JointAxis`, the masks and the limits conversion.
#[test]
fn test_axis_mask_and_limits_conversions() {
    let axes = array![
        (JointAxis::LinX, 0_u8, LIN_X), (JointAxis::LinY, 1, LIN_Y), (JointAxis::AngX, 2, ANG_X),
    ];
    for (axis, index, mask) in axes {
        assert_eq!((axis.index(), axis.mask()), (index, mask));
        assert_eq!(Into::<_, JointAxesMask>::into(axis), mask);
    }
    assert_eq!(Default::<JointAxesMask>::default().bits, 0);
    let limits: JointLimits = [NEG_HALF, ONE].into();
    assert_eq!(limits, JointLimits { min: NEG_HALF, max: ONE, impulse: ZERO });
}

/// Builder additions: `locked_axes`, `local_axis1/2`.
#[test]
fn test_builder_additions() {
    let b = GenericJointBuilderTrait::new(LIN_X)
        .locked_axes(LOCKED_REVOLUTE_AXES)
        .local_axis1(Y_AXIS)
        .local_axis2(Y_AXIS)
        .build();
    let mut expected: GenericJoint = RevoluteJointTrait::new().into();
    expected.set_local_axis1(Y_AXIS);
    expected.set_local_axis2(Y_AXIS);
    assert_eq!(b, expected);
    let p = PrismaticJointBuilderTrait::new(X_AXIS).local_axis1(Y_AXIS).local_axis2(Y_AXIS).build();
    assert_eq!(p, PrismaticJointBuilderTrait::new(Y_AXIS).build());
    let s = PinSlotJointBuilderTrait::new(X_AXIS)
        .local_axis1(Y_AXIS)
        .local_anchor1(v(HALF, HALF))
        .local_anchor2(v(ONE, ONE))
        .contacts_enabled(false)
        .limits([ZERO, ONE])
        .motor_model(MotorModel::ForceBased)
        .motor_max_force(HALF)
        .motor_velocity(ONE, HALF)
        .motor_position(ONE, HALF, HALF)
        .set_motor(ONE, HALF, HALF, HALF)
        .softness(soft())
        .build();
    let mut slot = PinSlotJointTrait::new(X_AXIS);
    slot.set_local_axis1(Y_AXIS);
    slot.set_local_anchor1(v(HALF, HALF));
    slot.set_local_anchor2(v(ONE, ONE));
    slot.set_contacts_enabled(false);
    slot.set_limits([ZERO, ONE]);
    slot.set_motor_model(MotorModel::ForceBased);
    slot.set_motor_max_force(HALF);
    slot.set_motor(ONE, HALF, HALF, HALF);
    slot.set_softness(soft());
    assert_eq!(s, slot.data());
}

#[test]
#[should_panic(expected: 'Joint: nonunit axis')]
fn test_complete_ang_frame_rejects_nonunit_axis() {
    let _ = GenericJointTrait::complete_ang_frame(v(ONE, ONE));
}

#[test]
#[should_panic(expected: 'Joint: nonunit axis')]
fn test_pin_slot_rejects_nonunit_axis() {
    let _ = PinSlotJointTrait::new(v(ZERO, ZERO));
}

#[test]
#[should_panic(expected: 'Joint: invalid mask')]
fn test_generic_joint_new_rejects_invalid_mask() {
    let _ = GenericJointTrait::new(JointAxesMask { bits: 8 });
}

#[test]
#[should_panic(expected: 'Joint: invalid axis')]
fn test_limits_reject_invalid_axis() {
    let j: GenericJoint = Default::default();
    let _ = j.limits(3);
}

fn three_joints() -> (ImpulseJointSet, Handle, Handle, Handle) {
    let mut set = ImpulseJointSetTrait::new();
    let disabled = GenericJoint { enabled: JointEnabled::Disabled, ..Default::default() };
    let h01 = set.insert(body(0), body(1), Default::default());
    let h12 = set.insert(body(1), body(2), disabled);
    let h10 = set.insert(body(1), body(0), Default::default());
    (set, h01, h12, h10)
}

/// The set queries, including after removals: stale handles resolve to nothing, a reused slot
/// keeps ascending-slot order and bumps the generation.
#[test]
fn test_set_queries_and_removals() {
    let (mut set, h01, h12, h10) = three_joints();
    assert!(!set.is_empty());
    assert!(set.contains(h01) && set.contains(h12) && set.contains(h10));
    assert!(!set.contains(Handle { index: 7, generation: 0 }));
    let joint = set.get(h12).unwrap();
    assert_eq!((joint.body1(), joint.body2()), (body(1), body(2)));
    assert_eq!(set.iter().len(), 3);
    // attached_joints: (body1, body2, handle, joint), ascending slot.
    let attached = set.attached_joints(body(1));
    assert_eq!(attached.len(), 3);
    let (b1, b2, h, _) = *attached.at(1);
    assert_eq!((b1, b2, h), (body(1), body(2), h12));
    assert_eq!(set.attached_joints(body(2)).len(), 1);
    assert_eq!(set.attached_joints(body(9)).len(), 0);
    // Only enabled ones: the disabled joint 1-2 is skipped.
    let enabled = set.attached_enabled_joints(body(1));
    assert_eq!(enabled.len(), 2);
    assert_eq!(set.attached_enabled_joints(body(2)).len(), 0);
    // joints_between: either order.
    let between = set.joints_between(body(0), body(1));
    assert_eq!(between.len(), 2);
    let (first, _) = *between.at(0);
    let (second, _) = *between.at(1);
    assert_eq!((first, second), (h01, h10));
    assert_eq!(set.joints_between(body(1), body(0)).len(), 2);
    assert_eq!(set.joints_between(body(0), body(2)).len(), 0);
    // get_unknown_gen: whatever the generation.
    let (j, handle) = set.get_unknown_gen(1).unwrap();
    assert_eq!((handle, j.body2), (h12, body(2)));
    assert!(set.get_unknown_gen(5).is_none());
    // Removal, then the same queries.
    assert!(set.remove(h01).is_some());
    assert!(!set.contains(h01) && set.get(h01).is_none());
    assert_eq!(set.joints_between(body(0), body(1)).len(), 1);
    assert!(set.get_unknown_gen(0).is_none());
    let reused = set.insert(body(3), body(2), Default::default());
    assert_eq!(reused.index, h01.index);
    assert!(reused.generation != h01.generation);
    assert!(!set.contains(h01) && set.contains(reused));
    let (handle, joint) = *set.iter().at(0);
    assert_eq!((handle, joint.body1), (reused, body(3)));
    let (_, handle) = set.get_unknown_gen(0).unwrap();
    assert_eq!(handle, reused);
    assert_eq!(set.attached_joints(body(2)).len(), 2);
}

/// `remove_joints_attached_to_rigid_body` and `set_bodies`, stale handles included.
#[test]
fn test_set_removal_of_attached_joints_and_rebinding() {
    let (mut set, h01, h12, h10) = three_joints();
    let removed = set.remove_joints_attached_to_rigid_body(body(0));
    assert_eq!(removed, array![h01, h10]);
    assert_eq!(set.len(), 1);
    assert!(!set.contains(h01) && !set.contains(h10) && set.contains(h12));
    assert_eq!(set.remove_joints_attached_to_rigid_body(body(0)).len(), 0);
    // set_bodies keeps the handle, the data and the impulses.
    let mut joint = set.get(h12).unwrap();
    joint.impulses = [HALF, ZERO, ONE];
    assert!(set.set(h12, joint));
    let rebound = set.set_bodies(h12, body(4), body(5)).unwrap();
    assert_eq!((rebound.body1, rebound.body2), (body(4), body(5)));
    assert_eq!(rebound.impulses, [HALF, ZERO, ONE]);
    assert_eq!(set.get(h12), Some(rebound));
    assert_eq!(set.attached_joints(body(2)).len(), 0);
    assert_eq!(set.attached_joints(body(5)).len(), 1);
    // A stale handle rebinds nothing.
    assert!(set.set_bodies(h01, body(4), body(5)).is_none());
    assert_eq!(set.len(), 1);
}

#[test]
fn gas_baseline() {
    let _ = opaque(ZERO);
}

#[test]
fn gas_generic_new() {
    let _ = opaque(GenericJointTrait::new(opaque(LOCKED_REVOLUTE_AXES)));
}

#[test]
fn gas_generic_lock_axes() {
    let mut j = opaque(Default::<GenericJoint>::default());
    j.lock_axes(opaque(ANG_X));
    let _ = opaque(j);
}

#[test]
fn gas_generic_set_enabled() {
    let mut j = opaque(Default::<GenericJoint>::default());
    j.set_enabled(opaque(false));
    let _ = opaque(j);
}

#[test]
fn gas_generic_set_local_axis1() {
    let mut j = opaque(Default::<GenericJoint>::default());
    j.set_local_axis1(opaque(Y_AXIS));
    let _ = opaque(j);
}

#[test]
fn gas_generic_local_axis1() {
    let _ = opaque(opaque(Default::<GenericJoint>::default()).local_axis1());
}

#[test]
fn gas_generic_limits() {
    let mut j = opaque(Default::<GenericJoint>::default());
    j.set_limits(2, [ZERO, ONE]);
    let _ = opaque(j.limits(opaque(2)));
}

#[test]
fn gas_generic_motor() {
    let mut j = opaque(Default::<GenericJoint>::default());
    j.set_motor_velocity(2, ONE, ONE);
    let _ = opaque(j.motor(opaque(2)));
}

#[test]
fn gas_generic_flip() {
    let mut j = opaque(Default::<GenericJoint>::default());
    j.flip();
    let _ = opaque(j);
}

#[test]
fn gas_typed_views() {
    let mut r = opaque(RevoluteJointTrait::new());
    r.set_limits(opaque([ZERO, ONE]));
    r.set_motor_velocity(opaque(ONE), opaque(HALF));
    r.set_local_anchor1(opaque(Y_AXIS));
    let _ = opaque((r.limits(), r.motor(), r.local_anchor1(), r.softness()));
    let _ = opaque(r.angle(opaque(Rot2 { re: ONE, im: ZERO }), opaque(Rot2 { re: ZERO, im: ONE })));
}

#[test]
fn gas_typed_new() {
    let _ = opaque(PrismaticJointTrait::new(opaque(X_AXIS)));
    let _ = opaque(PinSlotJointTrait::new(opaque(Y_AXIS)));
    let _ = opaque(RopeJointTrait::new(opaque(ONE)));
    let _ = opaque(SpringJointTrait::new(opaque(ONE), opaque(ONE), opaque(HALF)));
}

fn set_probe(op: u8) {
    let (mut set, h01, _, _) = three_joints();
    let b = opaque(body(1));
    match op {
        0 => { let _ = opaque(set.contains(opaque(h01))); },
        1 => { let _ = opaque(set.attached_joints(b)); },
        2 => { let _ = opaque(set.attached_enabled_joints(b)); },
        3 => { let _ = opaque(set.joints_between(b, opaque(body(0)))); },
        4 => { let _ = opaque(set.iter()); },
        5 => { let _ = opaque(set.get_unknown_gen(opaque(1))); },
        6 => { let _ = opaque(set.remove_joints_attached_to_rigid_body(b)); },
        7 => { let _ = opaque(set.set_bodies(opaque(h01), b, opaque(body(3)))); },
        _ => {},
    }
}

#[test]
fn gas_set_setup() {
    set_probe(8);
}

#[test]
fn gas_set_contains() {
    set_probe(0);
}

#[test]
fn gas_set_attached_joints() {
    set_probe(1);
}

#[test]
fn gas_set_attached_enabled_joints() {
    set_probe(2);
}

#[test]
fn gas_set_joints_between() {
    set_probe(3);
}

#[test]
fn gas_set_iter() {
    set_probe(4);
}

#[test]
fn gas_set_get_unknown_gen() {
    set_probe(5);
}

#[test]
fn gas_set_remove_attached() {
    set_probe(6);
}

#[test]
fn gas_set_set_bodies() {
    set_probe(7);
}

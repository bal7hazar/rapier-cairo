//! Tests and `gas_*` probes of the body and builder CCD members, and of the cold slot they share
//! with the user data.

use fixed::{HALF, ONE, ZERO};
use rapier_testing::opaque;
use crate::rigid_body::RigidBodyCcd;
use crate::rigid_body_set::{
    ColdExtraSlotTrait, RigidBody, RigidBodyBuilderTrait, RigidBodyCold, RigidBodyTrait,
    cold_or_default,
};
use super::{RigidBodyCcdApiTrait, body_ccd, wants_ccd};

#[test]
fn test_defaults_keep_no_cold_data() {
    let body = RigidBodyTrait::dynamic(Default::default());
    assert!(!body.is_ccd_enabled() && !body.is_ccd_active());
    assert_eq!(body.soft_ccd_prediction(), ZERO);
    assert_eq!(body.ccd(), Default::default());
    assert!(body.cold.unbox().is_none());
}

/// `(ccd_enabled, soft prediction, user data)` through the builder, then read back; the user
/// data and the CCD state share one slot and do not overwrite each other.
#[test]
fn test_builder_and_setters_round_trip() {
    let cases = array![(true, ZERO, 0_u128), (false, HALF, 7), (true, ONE, 99)];
    for (enabled, prediction, data) in cases {
        let body = RigidBodyBuilderTrait::dynamic()
            .user_data(data)
            .ccd_enabled(enabled)
            .soft_ccd_prediction(prediction)
            .build();
        assert_eq!(body.is_ccd_enabled(), enabled);
        assert_eq!(wants_ccd(@body), enabled);
        assert_eq!(body.soft_ccd_prediction(), prediction);
        assert_eq!(cold_or_default(body.cold).extra.value().user_data, data);
        let mut body = body;
        body.enable_ccd(!enabled);
        assert_eq!(body.is_ccd_enabled(), !enabled);
        assert_eq!(cold_or_default(body.cold).extra.value().user_data, data);
        let mut ccd = body.ccd();
        ccd.ccd_active = true;
        body.set_ccd(ccd);
        assert!(body.is_ccd_active());
        assert_eq!(body_ccd(@body).soft_ccd_prediction, prediction);
    }
}

#[test]
#[should_panic(expected: 'RigidBody: negative prediction')]
fn test_negative_soft_prediction_panics() {
    let mut body = RigidBodyTrait::dynamic(Default::default());
    body.set_soft_ccd_prediction(-HALF);
}

/// The cold value serializes the slot as an `Option`: null and set slots round-trip.
#[test]
fn test_cold_serde_round_trip() {
    let mut set = RigidBodyBuilderTrait::dynamic().ccd_enabled(true).build();
    set.set_soft_ccd_prediction(HALF);
    let null: RigidBodyCold = Default::default();
    let cases = array![null, cold_or_default(set.cold)];
    for cold in cases {
        let mut out = array![];
        cold.serialize(ref out);
        let mut span = out.span();
        let back: RigidBodyCold = Serde::deserialize(ref span).unwrap();
        assert_eq!(back, cold);
        assert!(span.is_empty());
    }
    assert!(null.extra.get().is_none());
    let ccd: RigidBodyCcd = cold_or_default(set.cold).extra.value().ccd;
    assert!(ccd.ccd_enabled);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_is_ccd_enabled_default() {
    let body: RigidBody = RigidBodyTrait::dynamic(opaque(Default::default()));
    let _ = body.is_ccd_enabled();
}

#[test]
fn gas_is_ccd_enabled_set() {
    let body = RigidBodyBuilderTrait::dynamic().ccd_enabled(opaque(true)).build();
    let _ = body.is_ccd_enabled();
}

#[test]
fn gas_enable_ccd() {
    let mut body: RigidBody = RigidBodyTrait::dynamic(opaque(Default::default()));
    body.enable_ccd(opaque(true));
}

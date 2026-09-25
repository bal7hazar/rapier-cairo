//! Kinematic and dominance scenes; old scene inputs and traces remain unchanged.
use super::*;

pub(super) fn scenes() -> Vec<SceneSpec> {
    vec![
        SceneSpec {
            id: "kinematic_platform", note: "position-based platform moves right/up carrying a frictional box; target is an integer Q32.32 increment per frame",
            bodies: vec![
                dynamic("platform", QPose::translation(0.0, 0.0), collider(ShapeSpec::cuboid(2.0, 0.5), 1.0, 0.0)),
                dynamic("box", QPose::translation(0.0, 1.0), collider(ShapeSpec::cuboid(0.5, 0.5), 1.0, 0.0)),
            ], joints: vec![], can_sleep: false,
        },
        SceneSpec {
            id: "kinematic_pusher", note: "velocity-based pusher moves at 1 m/s into a dynamic box on a frictionless ground",
            bodies: vec![ground(0.0, 0.0),
                dynamic("pusher", QPose::translation(-1.0, 0.5), collider(ShapeSpec::cuboid(0.5, 0.5), 0.0, 0.0)),
                dynamic("box", QPose::translation(0.0, 0.5), collider(ShapeSpec::cuboid(0.5, 0.5), 0.0, 0.0)),
            ], joints: vec![], can_sleep: false,
        },
        SceneSpec {
            id: "dominance_stack", note: "two dynamic boxes on ground; upper box dominance group 1, lower group 0",
            bodies: vec![ground(0.5, 0.0),
                dynamic("lower", QPose::translation(0.0, 0.5), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
                dynamic("upper", QPose::translation(0.0, 1.5), collider(ShapeSpec::cuboid(0.5, 0.5), 0.5, 0.0)),
            ], joints: vec![], can_sleep: false,
        },
    ]
}

pub(super) fn control(id: &str) -> Option<Value> {
    let (body, position_based, velocity, delta, dominant, group) = match id {
        "kinematic_platform" => (
            0,
            true,
            QVec::ZERO,
            QVec {
                x: Q(71582788),
                y: Q(35791394),
            },
            4,
            0,
        ),
        "kinematic_pusher" => (1, false, QVec::snap(1.0, 0.0), QVec::ZERO, 4, 0),
        "dominance_stack" => (4, false, QVec::ZERO, QVec::ZERO, 2, 1),
        _ => return None,
    };
    Some(
        json!({"kinematic_body": body, "position_based": position_based,
        "velocity": jqvec(velocity), "target_delta": jqvec(delta),
        "dominance_body": dominant, "dominance_group": group}),
    )
}

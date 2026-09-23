//! Dense solver storage independent of rigid-body components.
use fixed::Fixed;
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_math::pose2::Pose2;

/// Per-step body at its world centre of mass. Q32.32 positions/velocities must fit `Fixed`;
/// `im` is componentwise inverse mass (axis locks), `ii` is inverse angular inertia, not its
/// square root. Both must be nonnegative. Zero masses describe an immovable body.
/// `handle` is resolved with its generation during constraint generation; array order is free.
/// DF must preserve that order until the constraints are discarded. Default is a static body.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct SolverBody {
    pub handle: Handle,
    pub position: Pose2,
    pub linvel: Vec2,
    pub angvel: Fixed,
    pub im: Vec2,
    pub ii: Fixed,
}

/// Linear and physical angular velocity gathered once per manifold; Q32.32, default zero.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct SolverVel {
    pub linear: Vec2,
    pub angular: Fixed,
}

/// Two gathered solver bodies in constraint order. WORLD is represented by the default body.
/// Only velocities change during a sweep; poses, handles and masses are retained exactly.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub(crate) struct BodyPair {
    pub first: SolverBody,
    pub second: SolverBody,
}

pub(crate) const WORLD: u32 = 0xffffffff;

pub(crate) fn read(bodies: Span<SolverBody>, id: u32) -> SolverBody {
    if id == WORLD {
        Default::default()
    } else {
        *bodies.at(id)
    }
}

pub(crate) fn velocity(body: SolverBody) -> SolverVel {
    SolverVel { linear: body.linvel, angular: body.angvel }
}

// Cairo arrays are immutable. Rebuild once per manifold, retaining all non-velocity fields.
// This O(B) adapter is explicit: a mutable dense store belongs to DF, not a hidden dict here.
pub(crate) fn scatter(
    ref bodies: Array<SolverBody>, id1: u32, v1: SolverVel, id2: u32, v2: SolverVel,
) {
    let mut out = array![];
    let mut i = 0;
    while let Some(mut body) = bodies.pop_front() {
        if i == id1 {
            body.linvel = v1.linear;
            body.angvel = v1.angular;
        }
        if i == id2 {
            body.linvel = v2.linear;
            body.angvel = v2.angular;
        }
        out.append(body);
        i += 1;
    }
    bodies = out;
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, ZERO};
    use rapier_testing::opaque;
    use super::{SolverBody, SolverVel, WORLD, read, scatter, velocity};

    #[test]
    fn test_scatter_preserves_pose_and_world() {
        let b = SolverBody { angvel: ONE, ..Default::default() };
        let mut bodies = array![b, b, b];
        let v = SolverVel { angular: -ONE, ..Default::default() };
        scatter(ref bodies, 1, v, WORLD, v);
        assert_eq!(*bodies.at(0), b);
        assert_eq!(bodies.at(1).position, @b.position);
        assert_eq!(*bodies.at(2), b);
        assert_eq!(velocity(read(bodies.span(), 1)), v);
        assert_eq!(read(bodies.span(), WORLD).angvel, ZERO);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_default_body_and_velocity() {
        let b: SolverBody = Default::default();
        let v: SolverVel = Default::default();
        let _ = opaque((b, v));
    }
}

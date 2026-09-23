//! Scalar constraint rows. Products floor; effective-mass inversion rounds to nearest (zero maps to
//! zero). Every intermediate/output must fit Q32.32, otherwise fixed/core overflow panics.
use fixed::wide::{dot2, dot4};
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_math::math_ext::{gcross_vv, inv};
use super::super::body::{SolverBody, SolverVel};

/// Nonpenetration row; `gcross1/2` are signed lever-arm crosses with the force on each body.
/// `ii_gcross*` include inverse inertia. `r` is inverse effective mass, precomputed once.
/// `impulse >= 0`; the accumulator banks completed substeps, excluding last step's warm start.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactConstraintNormalPart {
    pub gcross1: Fixed,
    pub gcross2: Fixed,
    pub ii_gcross1: Fixed,
    pub ii_gcross2: Fixed,
    pub rhs: Fixed,
    pub rhs_wo_bias: Fixed,
    pub impulse: Fixed,
    pub impulse_accumulator: Fixed,
    pub r: Fixed,
    /// Per-point factor: speculative points use one even when the manifold is soft.
    pub cfm_factor: Fixed,
}

/// Single 2D Coulomb tangent row, clamped to +/- friction times current normal impulse.
/// Angular coefficients, mass and accumulation have the same units as the normal row.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactConstraintTangentPart {
    pub gcross1: Fixed,
    pub gcross2: Fixed,
    pub ii_gcross1: Fixed,
    pub ii_gcross2: Fixed,
    pub rhs: Fixed,
    pub rhs_wo_bias: Fixed,
    pub impulse: Fixed,
    pub impulse_accumulator: Fixed,
    pub r: Fixed,
}

/// Contact row pair plus frozen body-local anchors. `dist` is the separation offset such that
/// `dist + dot(world_p1 - world_p2, dir1)` reproduces the supplied solver-contact distance.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct ContactConstraintElement {
    pub normal_part: ContactConstraintNormalPart,
    pub tangent_part: ContactConstraintTangentPart,
    pub local_p1: Vec2,
    pub local_p2: Vec2,
    pub dist: Fixed,
    /// Captured restitution * approach velocity; negative seeds may bounce after substeps.
    pub restitution_seed: Fixed,
    /// Tracked point index, with `NEW_CONTACT_BIT` removed.
    pub contact_id: u8,
    /// Preserved for the future conveyor implementation; currently contributes zero.
    pub tangent_velocity: Vec2,
}

#[generate_trait]
pub impl ContactConstraintNormalPartImpl of ContactConstraintNormalPartTrait {
    /// Sum of impulses applied this step, including the final substep. Exact addition;
    /// panics on Q32.32 overflow.
    fn total_impulse(self: ContactConstraintNormalPart) -> Fixed {
        self.impulse_accumulator + self.impulse
    }
}

#[generate_trait]
pub impl ContactConstraintTangentPartImpl of ContactConstraintTangentPartTrait {
    /// Signed sum over this step, including its final substep. Exact addition, overflow panics.
    fn total_impulse(self: ContactConstraintTangentPart) -> Fixed {
        self.impulse_accumulator + self.impulse
    }
}

pub(crate) fn coefficients(
    dir: Vec2, a1: Vec2, a2: Vec2, b1: SolverBody, b2: SolverBody,
) -> (Fixed, Fixed, Fixed, Fixed, Fixed) {
    let g1 = gcross_vv(a1.x, a1.y, dir.x, dir.y);
    let g2 = gcross_vv(a2.x, a2.y, -dir.x, -dir.y);
    let ig1 = b1.ii * g1;
    let ig2 = b2.ii * g2;
    let mass_dir = (b1.im + b2.im) * dir;
    let k = dot4(dir.x, mass_dir.x, dir.y, mass_dir.y, g1, ig1, g2, ig2);
    (g1, g2, ig1, ig2, inv(k))
}

pub(crate) fn jv(dir: Vec2, g1: Fixed, g2: Fixed, v1: SolverVel, v2: SolverVel) -> Fixed {
    let dv = v1.linear - v2.linear;
    dot4(dir.x, dv.x, dir.y, dv.y, g1, v1.angular, g2, v2.angular)
}

pub(crate) fn apply(
    dir: Vec2,
    im1: Vec2,
    im2: Vec2,
    ig1: Fixed,
    ig2: Fixed,
    impulse: Fixed,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    v1.linear = v1.linear + (dir * im1) * Vec2 { x: impulse, y: impulse };
    v2.linear = v2.linear + (dir * im2) * Vec2 { x: -impulse, y: -impulse };
    v1.angular += ig1 * impulse;
    v2.angular += ig2 * impulse;
}

pub(crate) fn solve_normal(
    ref p: ContactConstraintNormalPart,
    dir: Vec2,
    im1: Vec2,
    im2: Vec2,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    let dv = jv(dir, p.gcross1, p.gcross2, v1, v2) + p.rhs;
    let new_impulse = p.cfm_factor * max(ZERO, p.impulse - p.r * dv);
    let delta = new_impulse - p.impulse;
    p.impulse = new_impulse;
    apply(dir, im1, im2, p.ii_gcross1, p.ii_gcross2, delta, ref v1, ref v2);
}

pub(crate) fn solve_tangent(
    ref p: ContactConstraintTangentPart,
    dir: Vec2,
    im1: Vec2,
    im2: Vec2,
    limit: Fixed,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    let dv = jv(dir, p.gcross1, p.gcross2, v1, v2) + p.rhs;
    let new_impulse = min(limit, max(-limit, p.impulse - p.r * dv));
    let delta = new_impulse - p.impulse;
    p.impulse = new_impulse;
    apply(dir, im1, im2, p.ii_gcross1, p.ii_gcross2, delta, ref v1, ref v2);
}

pub(crate) fn bounce(
    ref e: ContactConstraintElement,
    dir: Vec2,
    im1: Vec2,
    im2: Vec2,
    ref v1: SolverVel,
    ref v2: SolverVel,
) {
    if e.restitution_seed < ZERO && e.normal_part.total_impulse() > ZERO {
        let rhs = e.normal_part.rhs;
        let cfm = e.normal_part.cfm_factor;
        e.normal_part.rhs = e.restitution_seed;
        e.normal_part.cfm_factor = ONE;
        solve_normal(ref e.normal_part, dir, im1, im2, ref v1, ref v2);
        e.normal_part.rhs = rhs;
        e.normal_part.cfm_factor = cfm;
    }
}

pub(crate) fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}
pub(crate) fn tangent(dir: Vec2) -> Vec2 {
    Vec2 { x: -dir.y, y: dir.x }
}
pub(crate) fn max(a: Fixed, b: Fixed) -> Fixed {
    if a > b {
        a
    } else {
        b
    }
}
pub(crate) fn min(a: Fixed, b: Fixed) -> Fixed {
    if a < b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, TWO};
    use rapier_testing::opaque;
    use super::{
        ContactConstraintNormalPart, ContactConstraintNormalPartTrait, ContactConstraintTangentPart,
        ContactConstraintTangentPartTrait,
    };

    #[test]
    fn test_total_impulse() {
        let n = ContactConstraintNormalPart {
            impulse: TWO, impulse_accumulator: -ONE, ..Default::default(),
        };
        let t = ContactConstraintTangentPart {
            impulse: -TWO, impulse_accumulator: ONE, ..Default::default(),
        };
        assert_eq!(n.total_impulse(), ONE);
        assert_eq!(t.total_impulse(), -ONE);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_normal_total_impulse() {
        let _ = ContactConstraintNormalPart {
            impulse: opaque(TWO), impulse_accumulator: opaque(-ONE), ..Default::default(),
        }
            .total_impulse();
    }
    #[test]
    fn gas_tangent_total_impulse() {
        let _ = ContactConstraintTangentPart {
            impulse: opaque(TWO), impulse_accumulator: opaque(-ONE), ..Default::default(),
        }
            .total_impulse();
    }
}

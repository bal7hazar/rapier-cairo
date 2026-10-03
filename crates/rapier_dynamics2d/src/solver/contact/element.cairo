//! Scalar constraint rows. Products floor; effective-mass inversion rounds to nearest (zero maps to
//! zero). Every intermediate/output must fit Q32.32, otherwise fixed/core overflow panics.
//!
//! FU1: three chains of rescales are fused into one exact wide sum each, shared by this reference
//! path and the split sweeps (`island::sweeps::split`), which stay bit-identical to each other:
//! [`row_impulse`] (P1), [`separation`] (P2) and [`effective_mass`] (P3). Each is within one ulp
//! of the exact value of its formula (the unfused chains were up to 2.5 ulp off); the unfused forms
//! stay in [`alternatives`].
use fixed::wide::{WideAdd, WideLift, WideMul, WideNarrow, WideSub, dot2, wide_from, wide_mul};
use fixed::{Fixed, ONE, ZERO};
use glam_core::Vec2;
use rapier_math::math_ext::{gcross_vv, inv};
use rapier_math::pose2::Pose2;
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
    (g1, g2, ig1, ig2, inv(effective_mass(dir, b1.im + b2.im, g1, ig1, g2, ig2)))
}

/// The projected mass `dir . (im_sum * dir) + g1 ig1 + g2 ig2` with one floor (FU1 P3): the
/// linear terms `dir.x^2 im_sum.x + dir.y^2 im_sum.y` are exact triple products and the angular
/// ones exact products lifted to the same scale, so the sum is exact before its floor (within one
/// ulp of the formula; the unfused `dot4` of the floored `im_sum * dir` was up to 2.3 ulp off).
/// Panics with `'Fixed: overflow'` if the result does not fit.
#[inline(always)]
pub(crate) fn effective_mass(
    dir: Vec2, im_sum: Vec2, g1: Fixed, ig1: Fixed, g2: Fixed, ig2: Fixed,
) -> Fixed {
    wide_mul(dir.x, dir.x)
        .mul(im_sum.x)
        .add(wide_mul(dir.y, dir.y).mul(im_sum.y))
        .add(wide_mul(g1, ig1).lift())
        .add(wide_mul(g2, ig2).lift())
        .narrow()
}

pub(crate) fn jv(dir: Vec2, g1: Fixed, g2: Fixed, v1: SolverVel, v2: SolverVel) -> Fixed {
    let dv = v1.linear - v2.linear;
    fixed::wide::dot4(dir.x, dv.x, dir.y, dv.y, g1, v1.angular, g2, v2.angular)
}

/// The row's new unclamped impulse `impulse - r * (jv(..) + rhs)` with one floor (FU1 P1): the
/// velocity term is the exact wide sum of `jv` plus `rhs` (the velocity difference distributed
/// over it, `d * (a - b) = d * a - d * b` exactly), multiplied by `r` at the triple-product scale,
/// `impulse` lifted to it. Within one ulp of the formula, rounded down (the unfused
/// `impulse - floor(r * floor(jv + rhs))` was up to 2.5 ulp off, rounded up). Panics with
/// `'Fixed: overflow'` if the result does not fit.
#[inline(always)]
pub(crate) fn row_impulse(
    dir: Vec2,
    g1: Fixed,
    g2: Fixed,
    v1: SolverVel,
    v2: SolverVel,
    rhs: Fixed,
    impulse: Fixed,
    r: Fixed,
) -> Fixed {
    wide_from(impulse)
        .lift()
        .sub(
            wide_mul(dir.x, v1.linear.x)
                .sub(wide_mul(dir.x, v2.linear.x))
                .add(wide_mul(dir.y, v1.linear.y))
                .sub(wide_mul(dir.y, v2.linear.y))
                .add(wide_mul(g1, v1.angular))
                .add(wide_mul(g2, v2.angular))
                .add(wide_from(rhs))
                .mul(r),
        )
        .narrow()
}

/// The separations `(dist + (p1 l1 - p2 l2) . dir, (p1 l1 - p2 l2) . t)` of two body-local anchors
/// at the poses `p1`, `p2`, with one floor each (FU1 P2): the world difference stays an exact sum
/// per component (four products and two translations) and is projected on `dir` and `t` at the
/// triple-product scale. Within one ulp of the formula (the two floored transforms then the
/// floored dots were up to 2.3 ulp off). An identity pose is exact (`ONE * l` is `l` at the wide
/// scale), so a world endpoint needs no special case. Panics with `'Fixed: overflow'` if a
/// result does not fit.
#[inline(always)]
pub(crate) fn separation(
    p1: Pose2, l1: Vec2, p2: Pose2, l2: Vec2, dir: Vec2, t: Vec2, dist: Fixed,
) -> (Fixed, Fixed) {
    let (r1, r2) = (p1.rotation, p2.rotation);
    let dx = wide_mul(r1.re, l1.x)
        .sub(wide_mul(r1.im, l1.y))
        .add(wide_from(p1.translation.x))
        .sub(wide_mul(r2.re, l2.x))
        .add(wide_mul(r2.im, l2.y))
        .sub(wide_from(p2.translation.x));
    let dy = wide_mul(r1.im, l1.x)
        .add(wide_mul(r1.re, l1.y))
        .add(wide_from(p1.translation.y))
        .sub(wide_mul(r2.im, l2.x))
        .sub(wide_mul(r2.re, l2.y))
        .sub(wide_from(p2.translation.y));
    (
        dx.mul(dir.x).add(dy.mul(dir.y)).add(wide_from(dist).lift()).narrow(),
        dx.mul(t.x).add(dy.mul(t.y)).narrow(),
    )
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
    let new_impulse = p.cfm_factor
        * max(ZERO, row_impulse(dir, p.gcross1, p.gcross2, v1, v2, p.rhs, p.impulse, p.r));
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
    let new_impulse = min(
        limit, max(-limit, row_impulse(dir, p.gcross1, p.gcross2, v1, v2, p.rhs, p.impulse, p.r)),
    );
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

/// FU1's rejected (unfused) forms, kept for re-ranking (`tests::gas_*_unfused`) and as the
/// pre-FU1 reference of `tests::test_fused_rows_against_unfused`.
#[cfg(test)]
pub(crate) mod alternatives {
    use fixed::Fixed;
    use fixed::wide::{WideAdd, WideNarrow, WideSub, dot2_add, dot4, wide_from, wide_mul};
    use glam_core::Vec2;
    use rapier_math::pose2::Pose2;
    use super::super::super::body::SolverVel;

    /// P1 before FU1: BT3's `split::jv_add` (`jv + rhs` floored once), then `impulse - r * dv`
    /// floored again and subtracted (rounds the new impulse up).
    pub(crate) fn row_impulse_unfused(
        dir: Vec2,
        g1: Fixed,
        g2: Fixed,
        v1: SolverVel,
        v2: SolverVel,
        rhs: Fixed,
        impulse: Fixed,
        r: Fixed,
    ) -> Fixed {
        let dv = wide_mul(dir.x, v1.linear.x)
            .sub(wide_mul(dir.x, v2.linear.x))
            .add(wide_mul(dir.y, v1.linear.y))
            .sub(wide_mul(dir.y, v2.linear.y))
            .add(wide_mul(g1, v1.angular))
            .add(wide_mul(g2, v2.angular))
            .add(wide_from(rhs))
            .narrow();
        impulse - r * dv
    }

    /// `Pose2::transform_point` inlined (BT3), one floor per component.
    fn transform(p: Pose2, l: Vec2) -> Vec2 {
        let r = p.rotation;
        Vec2 {
            x: wide_mul(r.re, l.x)
                .sub(wide_mul(r.im, l.y))
                .add(wide_from(p.translation.x))
                .narrow(),
            y: dot2_add(r.im, l.x, r.re, l.y, p.translation.y),
        }
    }

    /// P2 before FU1: both anchors transformed and floored (BT3's `split::transform`), then the
    /// two dots of their difference (`split::separation`): six floors.
    pub(crate) fn separation_of_transforms(
        p1: Pose2, l1: Vec2, p2: Pose2, l2: Vec2, dir: Vec2, t: Vec2, dist: Fixed,
    ) -> (Fixed, Fixed) {
        let (a, b) = (transform(p1, l1), transform(p2, l2));
        (
            wide_mul(a.x, dir.x)
                .sub(wide_mul(b.x, dir.x))
                .add(wide_mul(a.y, dir.y))
                .sub(wide_mul(b.y, dir.y))
                .add(wide_from(dist))
                .narrow(),
            wide_mul(a.x, t.x)
                .sub(wide_mul(b.x, t.x))
                .add(wide_mul(a.y, t.y))
                .sub(wide_mul(b.y, t.y))
                .narrow(),
        )
    }

    /// P3 before FU1: `im_sum * dir` floored per component, then `dot4`.
    pub(crate) fn effective_mass_unfused(
        dir: Vec2, im_sum: Vec2, g1: Fixed, ig1: Fixed, g2: Fixed, ig2: Fixed,
    ) -> Fixed {
        let mass_dir = im_sum * dir;
        dot4(dir.x, mass_dir.x, dir.y, mass_dir.y, g1, ig1, g2, ig2)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, TWO};
    use glam_core::Vec2;
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::alternatives::{
        effective_mass_unfused, row_impulse_unfused, separation_of_transforms,
    };
    use super::super::super::body::SolverVel;
    use super::{
        ContactConstraintNormalPart, ContactConstraintNormalPartTrait, ContactConstraintTangentPart,
        ContactConstraintTangentPartTrait, effective_mass, row_impulse, separation, tangent,
    };

    fn f(raw: i64) -> Fixed {
        Fixed { raw }
    }
    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }
    fn pose(re: i64, im: i64, tx: i64, ty: i64) -> Pose2 {
        Pose2 { translation: v(tx, ty), rotation: Rot2 { re: f(re), im: f(im) } }
    }

    /// FU1: the fused rows against their unfused forms, on pile10-range inputs whose raw outputs
    /// come from an exact integer simulation of both (`docs/research/fused-rescales.md` §5): the
    /// fused result is within one ulp of the formula, the unfused one up to 2.5 ulp off.
    #[test]
    fn test_fused_rows_against_unfused() {
        // P1: (dx, dy, g1, g2, v1x, v1y, w1, v2x, v2y, w2, rhs, impulse, r, unfused, fused)
        let p1: Array<(i64, i64, i64, i64, i64, i64, i64, i64, i64, i64, i64, i64, i64, i64, i64)> =
            array![
            (
                3738854778,
                2113695582,
                -2408164469,
                -2554200033,
                31501791968,
                33217405706,
                -62995585696,
                -27601075561,
                63669456493,
                -414812685,
                -12001710194,
                9639895045,
                3920342447,
                -45154056471,
                -45154056472,
            ),
            (
                1310253315,
                4090229861,
                -1581610161,
                233374195,
                30622936609,
                45243116169,
                5355508110,
                -932396890,
                42777070800,
                -34557147231,
                1293424806,
                12766351301,
                6224022314,
                -882459883,
                -882459885,
            ),
            (
                -2105732591,
                3743345339,
                964327419,
                2136836372,
                -48933202163,
                -35480311284,
                -11672962052,
                -28509854336,
                33296746237,
                17972237285,
                -12647307292,
                869079469,
                3190221469,
                42655864987,
                42655864986,
            ),
            (
                -4006420556,
                -1547688084,
                -1593499536,
                3208495683,
                44921510699,
                5676003137,
                53486064549,
                5238057736,
                52253169973,
                4227614592,
                18954164784,
                11393715705,
                1892079627,
                19307993808,
                19307993807,
            ),
            (
                456914046,
                4270594060,
                -2660345324,
                2374676511,
                -55172191373,
                40590405774,
                53786604050,
                35828857717,
                -11563582556,
                -47654089698,
                -8315405229,
                8944128812,
                795297431,
                13721908795,
                13721908794,
            ),
            (
                -3343224852,
                -2696218030,
                2060287026,
                -2037639611,
                24145972088,
                50463183327,
                39002285637,
                -19510345943,
                3669451431,
                45062867092,
                -19866499355,
                5256310590,
                4056434996,
                86379744889,
                86379744888,
            ),
        ];
        for (dx, dy, g1, g2, v1x, v1y, w1, v2x, v2y, w2, rhs, imp, r, unfused, fused) in p1 {
            let (dir, a, b) = (
                v(dx, dy),
                SolverVel { linear: v(v1x, v1y), angular: f(w1) },
                SolverVel { linear: v(v2x, v2y), angular: f(w2) },
            );
            let (g1, g2, rhs, imp, r) = (f(g1), f(g2), f(rhs), f(imp), f(r));
            assert_eq!(row_impulse_unfused(dir, g1, g2, a, b, rhs, imp, r).raw, unfused);
            assert_eq!(row_impulse(dir, g1, g2, a, b, rhs, imp, r).raw, fused);
        }
        // P2: ((re1, im1, t1x, t1y), (re2, im2, t2x, t2y), (l1x, l1y, l2x, l2y), (dx, dy, dist),
        // (unfused n, unfused t, fused n, fused t)); `re2 = ONE` rows are a world endpoint (nested:
        // `Drop` is implemented for tuples of at most 16 elements).
        let p2: Array<
            (
                (i64, i64, i64, i64),
                (i64, i64, i64, i64),
                (i64, i64, i64, i64),
                (i64, i64, i64),
                (i64, i64, i64, i64),
            ),
        > =
            array![
            (
                (4294468045, -65484952, 77209153462, 12412650721),
                (4294967296, 0, 0, 0),
                (30575387, -9966549, 78446950487, 10919881866),
                (3978274063, 1618665977, -395568916),
                (-955263115, 1828066140, -955263114, 1828066140),
            ),
            (
                (3939783742, -1710218739, 89447658365, 3387063995),
                (4037798294, -1463874657, 83109497883, 10887959763),
                (-552257735, -76038973, 1626631217, 210412969),
                (-4288834850, -229433445, 201402404),
                (-3619324436, 7208536145, -3619324436, 7208536144),
            ),
            (
                (4181046010, 982648629, 90892417001, 11101772726),
                (4294967296, 0, 0, 0),
                (-2014817683, 1668037737, 92513515616, 11002540290),
                (-239609163, -4288278387, 51802151),
                (-987136342, -4028340419, -987136342, -4028340418),
            ),
            (
                (3974142099, 1628784408, 82782815425, 16113306799),
                (4243792876, -661034111, 74064002176, 745873339),
                (-244171907, -1032646230, -1569881511, 1497850113),
                (-4093099630, -1301260734, -345629219),
                (-13887882435, -8913715270, -13887882436, -8913715270),
            ),
            (
                (3972168463, -1633591680, 81798363783, 16684856417),
                (4294967296, 0, 0, 0),
                (-86208032, 832142468, 82823189199, 14709539805),
                (-1977939420, 3812413897, -71566730),
                (2756971663, -579694175, 2756971662, -579694176),
            ),
            (
                (4205600410, -871590076, 75535688918, 7974620839),
                (4294967296, 0, 0, 0),
                (63440309, 249343381, 76393643607, 9364336731),
                (2066516406, -3765136654, -138755033),
                (518206007, -1210680704, 518206006, -1210680704),
            ),
        ];
        for (a, b, l, d, e) in p2 {
            let ((re1, im1, t1x, t1y), (re2, im2, t2x, t2y)) = (a, b);
            let ((l1x, l1y, l2x, l2y), (dx, dy, dist), (un, ut, fn_, ft)) = (l, d, e);
            let (p1, p2) = (pose(re1, im1, t1x, t1y), pose(re2, im2, t2x, t2y));
            let (l1, l2, dir) = (v(l1x, l1y), v(l2x, l2y), v(dx, dy));
            let (n, t) = separation_of_transforms(p1, l1, p2, l2, dir, tangent(dir), f(dist));
            assert_eq!((n.raw, t.raw), (un, ut));
            let (n, t) = separation(p1, l1, p2, l2, dir, tangent(dir), f(dist));
            assert_eq!((n.raw, t.raw), (fn_, ft));
        }
        // P3: (dx, dy, im.x, im.y, g1, g2, ig1, ig2, unfused, fused)
        let p3: Array<(i64, i64, i64, i64, i64, i64, i64, i64, i64, i64)> = array![
            (
                -587288016,
                4254625349,
                3069339598,
                3069339598,
                -603523133,
                1147105209,
                -2075012181,
                8609959303,
                5660476211,
                5660476212,
            ),
            (
                -2638746250,
                -3388770029,
                3361526234,
                3361526234,
                -1459424778,
                -2220265992,
                -11365927080,
                0,
                7223654737,
                7223654736,
            ),
            (
                -48452744,
                -4294693983,
                5414039185,
                5414039185,
                -714373729,
                -1021491433,
                -6432376247,
                -4022546871,
                7440624530,
                7440624529,
            ),
            (
                3342629245,
                -2696956396,
                2469061068,
                2469061068,
                34683281,
                1855618629,
                361909718,
                20167558244,
                11185273884,
                11185273885,
            ),
            (
                996645042,
                4177731768,
                9032399358,
                9032399358,
                1020370815,
                -586467619,
                4145800593,
                -1244834797,
                10187311343,
                10187311344,
            ),
            (
                4251937430,
                -606442215,
                8589934592,
                8589934592,
                1230007923,
                3120773054,
                14133136793,
                35062000655,
                38113888640,
                38113888640,
            ),
        ];
        for (dx, dy, imx, imy, g1, g2, ig1, ig2, unfused, fused) in p3 {
            let (dir, im) = (v(dx, dy), v(imx, imy));
            assert_eq!(effective_mass_unfused(dir, im, f(g1), f(ig1), f(g2), f(ig2)).raw, unfused);
            assert_eq!(effective_mass(dir, im, f(g1), f(ig1), f(g2), f(ig2)).raw, fused);
        }
    }

    const DIR: Vec2 = Vec2 { x: Fixed { raw: 3037000499 }, y: Fixed { raw: -3037000500 } };
    const V1: SolverVel = SolverVel {
        linear: Vec2 { x: Fixed { raw: 12884901888 }, y: Fixed { raw: -30064771072 } },
        angular: Fixed { raw: 4294967296 },
    };
    const V2: SolverVel = SolverVel {
        linear: Vec2 { x: Fixed { raw: -2147483648 }, y: Fixed { raw: 1288490189 } },
        angular: Fixed { raw: -6442450944 },
    };
    const P1: Pose2 = Pose2 {
        translation: Vec2 { x: Fixed { raw: 81604378624 }, y: Fixed { raw: 6438155977 } },
        rotation: Rot2 { re: Fixed { raw: 4273492023 }, im: Fixed { raw: 428782622 } },
    };
    const P2: Pose2 = Pose2 {
        translation: Vec2 { x: Fixed { raw: 83751862272 }, y: Fixed { raw: 2147483648 } },
        rotation: Rot2 { re: Fixed { raw: 4289601186 }, im: Fixed { raw: -214659224 } },
    };
    const L1: Vec2 = Vec2 { x: Fixed { raw: 2147483648 }, y: Fixed { raw: -2147483648 } };
    const L2: Vec2 = Vec2 { x: Fixed { raw: -2147483648 }, y: Fixed { raw: 2147483648 } };
    const IM: Vec2 = Vec2 { x: Fixed { raw: 8589934592 }, y: Fixed { raw: 8589934592 } };

    #[test]
    fn gas_row_impulse() {
        let (d, a, b) = opaque((DIR, V1, V2));
        let _ = row_impulse(d, ONE, -ONE, a, b, opaque(TWO), opaque(ONE), opaque(TWO));
    }
    #[test]
    fn gas_row_impulse_unfused() {
        let (d, a, b) = opaque((DIR, V1, V2));
        let _ = row_impulse_unfused(d, ONE, -ONE, a, b, opaque(TWO), opaque(ONE), opaque(TWO));
    }
    #[test]
    fn gas_separation() {
        let (p1, p2, l1, l2, d) = opaque((P1, P2, L1, L2, DIR));
        let _ = separation(p1, l1, p2, l2, d, tangent(d), opaque(ONE));
    }
    #[test]
    fn gas_separation_unfused() {
        let (p1, p2, l1, l2, d) = opaque((P1, P2, L1, L2, DIR));
        let _ = separation_of_transforms(p1, l1, p2, l2, d, tangent(d), opaque(ONE));
    }
    #[test]
    fn gas_effective_mass() {
        let (d, im) = opaque((DIR, IM));
        let _ = effective_mass(d, im, opaque(ONE), opaque(TWO), opaque(-ONE), opaque(ONE));
    }
    #[test]
    fn gas_effective_mass_unfused() {
        let (d, im) = opaque((DIR, IM));
        let _ = effective_mass_unfused(d, im, opaque(ONE), opaque(TWO), opaque(-ONE), opaque(ONE));
    }

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

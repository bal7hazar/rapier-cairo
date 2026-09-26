//! Mass properties of a shape (Parry's `MassProperties`, 2D).
//!
//! Frozen in `docs/interfaces/geometry-dynamics.md` §4; the type is fixed, the constructors per
//! shape, `transform_by` and the combination of two properties are work package GB.
//!
//! The disc/box/capsule formulas use fused kernels: exact wide products, one floor per output (the
//! composed-ops forms live in `alternatives` and are ranked by the `gas_*` probes). Inverses go
//! through `rapier_math::inv`, so a zero mass or inertia stays an infinite one.
//! `inv_mass == 0` means infinite mass, and `rapier_math::math_ext::inv` (`inv(0) = 0`) is what
//! makes fixed bodies work without special cases.

use core::num::traits::Zero;
use core::ops::{AddAssign, SubAssign};
use fixed::wide::{WideAdd, WideMul, WideNarrow, dot2, norm2, wide_mul};
use fixed::{Fixed, PI, ZERO};
use glam::vec2::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_math::{DEFAULT_EPSILON, inv};
use crate::point::cross_wide;
use crate::shape::{ConvexPolygon, ConvexPolygonTrait};

/// Centre of mass in the shape's local frame, inverse mass, inverse principal angular inertia
/// (a scalar in 2D).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct MassProperties {
    pub local_com: Vec2,
    pub inv_mass: Fixed,
    pub inv_principal_inertia: Fixed,
}

/// Divisors of the inertia formulas: applied as raw integer divisions of a value already floored
/// once (see `from_cuboid` and `from_capsule`), which is more accurate than a truncated `1 / 3`.
const THREE: i64 = 3;
const TWENTY_FOUR: i64 = 24;

/// `x / d` on the raw, rounding toward zero; `d` is a non-zero constant.
#[inline(always)]
fn div_raw(x: Fixed, d: i64) -> Fixed {
    Fixed { raw: x.raw / d }
}

/// `|shift|^2 * mass` with one rescale: the parallel-axis term.
#[inline(always)]
fn shift_term(shift: Vec2, mass: Fixed) -> Fixed {
    wide_mul(shift.x, shift.x).add(wide_mul(shift.y, shift.y)).mul(mass).narrow()
}

/// Mass and inertia of a disc. `pi * density` floors once, then `r^2 * (pi density)` and
/// `r^2 * mass` are exact wide products with one floor each.
pub(crate) fn ball_mass_inertia(density: Fixed, radius: Fixed) -> (Fixed, Fixed) {
    let r2 = wide_mul(radius, radius);
    let mass = r2.mul(PI * density).narrow();
    (mass, div_raw(r2.mul(mass).narrow(), 2))
}

/// Mass and inertia of a box. The mass is one exact wide product; `hx^2 + hy^2` is summed wide
/// before the multiplication by the mass, so the inertia costs two floors (the second is the
/// division by 3).
pub(crate) fn cuboid_mass_inertia(density: Fixed, he: Vec2) -> (Fixed, Fixed) {
    let mass = wide_mul(he.x, he.y).mul(Fixed { raw: density.raw * 4 }).narrow();
    let sq = wide_mul(he.x, he.x).add(wide_mul(he.y, he.y));
    (mass, div_raw(sq.mul(mass).narrow(), THREE))
}

/// Mass and inertia (about the midpoint) of a capsule of core length `len` and radius `r`.
///
/// With `bv = pi r^2` and `s = 4 r^2 + len^2` the inertia is
/// `density / 24 * (4 r len s + (12 r^2 + 6 len^2 + 9 len r) bv)`: the bracket is one exact wide
/// sum and one floor, then `* density` and `/ 24` (two more floors, the last one scaled down by
/// 24), and the mass is one wide sum with one floor.
pub(crate) fn capsule_mass_inertia(density: Fixed, len: Fixed, r: Fixed) -> (Fixed, Fixed) {
    let bv = wide_mul(r, r).mul(PI).narrow();
    let mass = wide_mul(r, len)
        .mul(Fixed { raw: density.raw * 2 })
        .add(wide_mul(r, r).mul(PI * density))
        .narrow();
    let r4 = Fixed { raw: r.raw * 4 };
    let s = wide_mul(r4, r).add(wide_mul(len, len)).narrow();
    let w = wide_mul(Fixed { raw: r.raw * 12 }, r)
        .add(wide_mul(Fixed { raw: len.raw * 6 }, len))
        .add(wide_mul(Fixed { raw: len.raw * 9 }, r));
    let x = w.mul(bv).add(wide_mul(r4, len).mul(s)).narrow();
    (mass, div_raw(x * density, TWENTY_FOUR))
}

#[generate_trait]
pub impl MassPropertiesImpl of MassPropertiesTrait {
    /// From a centre of mass, a mass and a principal inertia (`0` means infinite, as upstream).
    /// #### Panics
    /// * `'Fixed: overflow'` for a non-zero mass or inertia below `2^-31`, whose reciprocal is
    ///   not representable.
    #[inline(always)]
    fn new(local_com: Vec2, mass: Fixed, principal_inertia: Fixed) -> MassProperties {
        MassProperties {
            local_com, inv_mass: inv(mass), inv_principal_inertia: inv(principal_inertia),
        }
    }

    /// Properties of a disc of radius `radius`: `mass = pi r^2 density`, `I = mass r^2 / 2`.
    /// #### Panics
    /// * `'Fixed: overflow'` when the mass or inertia leaves the scalar range.
    fn from_ball(density: Fixed, radius: Fixed) -> MassProperties {
        let (mass, inertia) = ball_mass_inertia(density, radius);
        Self::new(Vec2 { x: ZERO, y: ZERO }, mass, inertia)
    }

    /// Properties of a box: `mass = 4 hx hy density`, `I = mass (hx^2 + hy^2) / 3`.
    /// #### Panics
    /// * `'i64_mul Overflow'` for a density of `2^61` or more, `'Fixed: overflow'` when the mass or
    ///   inertia leaves the scalar range.
    fn from_cuboid(density: Fixed, half_extents: Vec2) -> MassProperties {
        let (mass, inertia) = cuboid_mass_inertia(density, half_extents);
        Self::new(Vec2 { x: ZERO, y: ZERO }, mass, inertia)
    }

    /// Properties of the capsule of core segment `a`–`b` and radius `radius`: a `2 r` wide
    /// rectangle of length `L = |b - a|` plus a disc, about the midpoint of the segment (2D
    /// `from_capsule`, with the parallel-axis term of the caps folded in).
    /// #### Panics
    /// * `'Fixed: overflow'` when a value leaves the scalar range.
    fn from_capsule(density: Fixed, a: Vec2, b: Vec2, radius: Fixed) -> MassProperties {
        let (mass, inertia) = capsule_mass_inertia(density, norm2(b.x - a.x, b.y - a.y), radius);
        Self::new(a.midpoint(b), mass, inertia)
    }

    /// Triangle-fan polygon mass in upstream vertex accumulation order: geometric center,
    /// area-weighted center, then inertia about that center. Each product floors; raw constant
    /// divisions truncate toward zero. Inputs/intermediates and nonzero inverses must fit Q32.32.
    /// A sub-resolution area produces zero mass/inertia, retaining the geometric center.
    fn from_convex_polygon(density: Fixed, polygon: ConvexPolygon) -> MassProperties {
        let mut center = Vec2 { x: ZERO, y: ZERO };
        let mut i = 0;
        while i != polygon.count {
            center = center + polygon.vertex(i);
            i += 1;
        }
        let n: i64 = polygon.count.into();
        center = Vec2 { x: div_raw(center.x, n), y: div_raw(center.y, n) };
        let mut area = ZERO;
        let mut weighted = Vec2 { x: ZERO, y: ZERO };
        i = 0;
        while i != polygon.count {
            let a = polygon.vertex(i);
            let b = polygon.vertex(polygon.next(i));
            let e = b - a;
            let d = center - a;
            let raw = cross_wide(e.x, e.y, d.x, d.y) / 0x200000000;
            let part = Fixed {
                raw: raw.try_into().expect(crate::shape::convex_polygon::errors::AREA_OVERFLOW),
            };
            let sum = a + b + center;
            let centroid = Vec2 { x: div_raw(sum.x, 3), y: div_raw(sum.y, 3) };
            weighted = weighted + centroid.mul_scalar(part);
            area = area + part;
            i += 1;
        }
        if area == ZERO {
            return Self::new(center, ZERO, ZERO);
        }
        let com = Vec2 { x: weighted.x / area, y: weighted.y / area };
        let mut inertia = ZERO;
        i = 0;
        while i != polygon.count {
            let a = polygon.vertex(i) - com;
            let b = polygon.vertex(polygon.next(i)) - com;
            let raw = cross_wide(a.x, a.y, b.x, b.y) / 0x200000000;
            let part = Fixed {
                raw: raw.try_into().expect(crate::shape::convex_polygon::errors::AREA_OVERFLOW),
            };
            let unit = wide_mul(a.x, a.x)
                .add(wide_mul(b.x, a.x))
                .add(wide_mul(b.x, b.x))
                .add(wide_mul(a.y, a.y))
                .add(wide_mul(b.y, a.y))
                .add(wide_mul(b.y, b.y))
                .narrow();
            inertia = inertia + div_raw(unit, 6) * part;
            i += 1;
        }
        Self::new(com, area * density, inertia * density)
    }

    /// Properties of the triangle `a`, `b`, `c` (upstream `from_triangle`): mass `area *
    /// density` at the centre `(a + b + c) / 3`, inertia `unit_angular_inertia * area * density`
    /// (about `a`, as upstream). A zero area (exactly collinear vertices) gives zero mass and
    /// inertia at the centre. Area and inertia floor once each (see `TriangleTrait`).
    /// #### Panics
    /// * `'Fixed: overflow'` when a product or a non-zero inverse leaves the scalar range.
    fn from_triangle(density: Fixed, a: Vec2, b: Vec2, c: Vec2) -> MassProperties {
        let triangle = crate::shape::triangle::Triangle { a, b, c };
        let area = crate::shape::triangle::TriangleTrait::area(triangle);
        let com = crate::shape::triangle::TriangleTrait::center(triangle);
        if area == ZERO {
            return Self::new(com, ZERO, ZERO);
        }
        let ipart = crate::shape::triangle::TriangleTrait::unit_angular_inertia(triangle);
        Self::new(com, area * density, ipart * area * density)
    }

    /// Zero mass properties: a segment has no area (and a half-space is infinite).
    #[inline(always)]
    fn from_segment() -> MassProperties {
        Zero::<MassProperties>::zero()
    }

    /// `1 / inv_mass`, `0` for an infinite mass.
    #[inline(always)]
    fn mass(self: MassProperties) -> Fixed {
        inv(self.inv_mass)
    }

    /// `1 / inv_principal_inertia`, `0` for an infinite inertia.
    #[inline(always)]
    fn principal_inertia(self: MassProperties) -> Fixed {
        inv(self.inv_principal_inertia)
    }

    /// The centre of mass in the frame `pose` maps the local frame into.
    #[inline(always)]
    fn world_com(self: MassProperties, pose: Pose2) -> Vec2 {
        pose.transform_point(self.local_com)
    }

    /// The inverse angular inertia in world space: in 2D the scalar `inv_principal_inertia`,
    /// whatever the rotation (upstream 2D `world_inv_inertia`).
    #[inline(always)]
    fn world_inv_inertia(self: MassProperties, rot: Rot2) -> Fixed {
        self.inv_principal_inertia
    }

    /// The same properties expressed in the frame `pose` maps the local frame into: only the
    /// centre of mass moves (in 2D the inertia is a rotation-invariant scalar).
    #[inline(always)]
    fn transform_by(self: MassProperties, pose: Pose2) -> MassProperties {
        MassProperties { local_com: pose.transform_point(self.local_com), ..self }
    }

    /// Replaces the mass. With `adjust_angular_inertia` the inertia scales with it
    /// (`inv_I *= curr_mass / new_mass`, one wide triple product and one floor).
    /// #### Panics
    /// * `'Fixed: overflow'` as `new`.
    fn set_mass(ref self: MassProperties, new_mass: Fixed, adjust_angular_inertia: bool) {
        let new_inv_mass = inv(new_mass);
        if adjust_angular_inertia {
            let curr_mass = inv(self.inv_mass);
            self
                .inv_principal_inertia = wide_mul(new_inv_mass, curr_mass)
                .mul(self.inv_principal_inertia)
                .narrow();
        }
        self.inv_mass = new_inv_mass;
    }

    /// The same properties with the principal inertia replaced by `principal_inertia`.
    /// #### Panics
    /// * `'Fixed: overflow'` as `new`.
    #[inline(always)]
    fn with_inertia(self: MassProperties, principal_inertia: Fixed) -> MassProperties {
        MassProperties { inv_principal_inertia: inv(principal_inertia), ..self }
    }

    /// The same properties with the principal inertia multiplied by `scale` (`scale = 0` gives
    /// an infinite inertia).
    /// #### Panics
    /// * `'Fixed: overflow'` for a non-zero `scale` below `2^-31`.
    #[inline(always)]
    fn with_inertia_scaled(self: MassProperties, scale: Fixed) -> MassProperties {
        MassProperties { inv_principal_inertia: self.inv_principal_inertia * inv(scale), ..self }
    }

    /// Inertia about a point displaced by `shift` from the centre of mass (parallel-axis theorem):
    /// `I + |shift|^2 mass`, `I` alone when the mass is infinite.
    #[inline(always)]
    fn shifted_inertia(self: MassProperties, shift: Vec2) -> Fixed {
        shifted_inertia_of(self, inv(self.inv_mass), shift)
    }
}

/// `shifted_inertia` for a caller that already holds `mass = inv(inv_mass)` (an infinite mass is
/// `0`, whose parallel-axis term vanishes, so no branch is needed).
#[inline(always)]
fn shifted_inertia_of(p: MassProperties, mass: Fixed, shift: Vec2) -> Fixed {
    inv(p.inv_principal_inertia) + shift_term(shift, mass)
}

impl MassPropertiesZero of Zero<MassProperties> {
    #[inline(always)]
    fn zero() -> MassProperties {
        MassProperties {
            local_com: Vec2 { x: ZERO, y: ZERO }, inv_mass: ZERO, inv_principal_inertia: ZERO,
        }
    }
    #[inline(always)]
    fn is_zero(self: @MassProperties) -> bool {
        *self == Self::zero()
    }
    #[inline(always)]
    fn is_non_zero(self: @MassProperties) -> bool {
        !Self::is_zero(self)
    }
}

/// Combination of two rigid parts: masses add, the centre of mass is the mass-weighted mean and
/// the inertias, taken about it, add (parallel-axis theorem). A zero operand is the identity.
pub impl MassPropertiesAdd of Add<MassProperties> {
    fn add(lhs: MassProperties, rhs: MassProperties) -> MassProperties {
        if lhs.is_zero() {
            return rhs;
        } else if rhs.is_zero() {
            return lhs;
        }
        let m1 = inv(lhs.inv_mass);
        let m2 = inv(rhs.inv_mass);
        let inv_mass = inv(m1 + m2);
        let com = Vec2 {
            x: dot2(lhs.local_com.x, m1, rhs.local_com.x, m2) * inv_mass,
            y: dot2(lhs.local_com.y, m1, rhs.local_com.y, m2) * inv_mass,
        };
        let i1 = shifted_inertia_of(lhs, m1, com - lhs.local_com);
        let i2 = shifted_inertia_of(rhs, m2, com - rhs.local_com);
        MassProperties { local_com: com, inv_mass, inv_principal_inertia: inv(i1 + i2) }
    }
}

/// Removal of a part from a whole (upstream `Sub`): masses and inertias below `DEFAULT_EPSILON`
/// (`f32::EPSILON`) clamp to zero, i.e. to an infinite body.
pub impl MassPropertiesSub of Sub<MassProperties> {
    fn sub(lhs: MassProperties, rhs: MassProperties) -> MassProperties {
        if lhs.is_zero() || rhs.is_zero() {
            return lhs;
        }
        let m1 = inv(lhs.inv_mass);
        let m2 = inv(rhs.inv_mass);
        let mut new_mass = m1 - m2;
        if new_mass < DEFAULT_EPSILON {
            new_mass = ZERO;
        }
        let inv_mass = inv(new_mass);
        let com = Vec2 {
            x: (lhs.local_com.x * m1 - rhs.local_com.x * m2) * inv_mass,
            y: (lhs.local_com.y * m1 - rhs.local_com.y * m2) * inv_mass,
        };
        let i1 = shifted_inertia_of(lhs, m1, com - lhs.local_com);
        let i2 = shifted_inertia_of(rhs, m2, com - rhs.local_com);
        let mut inertia = i1 - i2;
        if inertia < DEFAULT_EPSILON {
            inertia = ZERO;
        }
        MassProperties { local_com: com, inv_mass, inv_principal_inertia: inv(inertia) }
    }
}

/// `self = self + rhs` (upstream `AddAssign`).
pub impl MassPropertiesAddAssign of AddAssign<MassProperties, MassProperties> {
    fn add_assign(ref self: MassProperties, rhs: MassProperties) {
        self = self + rhs;
    }
}

/// `self = self - rhs` (upstream `SubAssign`).
pub impl MassPropertiesSubAssign of SubAssign<MassProperties, MassProperties> {
    fn sub_assign(ref self: MassProperties, rhs: MassProperties) {
        self = self - rhs;
    }
}

/// Rejected candidates: the formulas as upstream writes them, one rounded `Fixed` operation at a
/// time. Kept for the `gas_*` ranking and as oracles for the fuzz tests.
#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, FixedTrait, PI};
    use glam::Vec2;

    fn three() -> Fixed {
        FixedTrait::from_int(3)
    }

    pub fn ball_mass_inertia(density: Fixed, r: Fixed) -> (Fixed, Fixed) {
        let mass = PI * r * r * density;
        (mass, r * r / FixedTrait::from_int(2) * mass)
    }

    pub fn cuboid_mass_inertia(density: Fixed, he: Vec2) -> (Fixed, Fixed) {
        let volume = he.x * he.y * FixedTrait::from_int(4);
        let unit = he.x * he.x / three() + he.y * he.y / three();
        let mass = volume * density;
        (mass, unit * mass)
    }

    /// Upstream order: cylinder (2D: a cuboid of half extents `(r, half_height)`) plus ball, then
    /// the parallel-axis term of the caps.
    pub fn capsule_mass_inertia(density: Fixed, len: Fixed, r: Fixed) -> (Fixed, Fixed) {
        let half_height = len / FixedTrait::from_int(2);
        let cyl_vol = r * half_height * FixedTrait::from_int(4);
        let cyl_unit = r * r / three() + half_height * half_height / three();
        let ball_vol = PI * r * r;
        let ball_unit = r * r / FixedTrait::from_int(2);
        let mass = (cyl_vol + ball_vol) * density;
        let h = half_height * FixedTrait::from_int(2);
        let quarter = FixedTrait::from_ratio(1, 4);
        let three_eighths = FixedTrait::from_ratio(3, 8);
        let extra = (h * h * quarter + h * r * three_eighths) * ball_vol * density;
        (mass, (cyl_unit * cyl_vol + ball_unit * ball_vol) * density + extra)
    }
}

#[cfg(test)]
mod tests {
    use core::num::traits::Zero;
    use fixed::wide::norm2;
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_golden::compare::abs_diff;
    use rapier_golden::mass_properties::cases;
    use rapier_golden::types::ShapeRaw;
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use super::{
        MassProperties, MassPropertiesTrait, alternatives, ball_mass_inertia, capsule_mass_inertia,
        cuboid_mass_inertia,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn i(n: i32) -> Fixed {
        FixedTrait::from_int(n)
    }

    fn near(a: Fixed, b: Fixed, ulps: i64) {
        assert!(a.abs_diff_eq(b, Fixed { raw: ulps }), "{:?} vs {:?}", a, b);
    }

    fn oracle_tolerance(value: Fixed, rounded_sites: i64) -> i64 {
        rounded_sites + value.raw / 0x10000000
    }

    /// `unit_cuboid_at(-1)` / `unit_cuboid_at(1)` as literals, so that the gas probes measure the
    /// operator and not the construction of its operands (`test_probe_constants` pins them).
    const LEFT: MassProperties = MassProperties {
        local_com: Vec2 { x: Fixed { raw: -4294967296 }, y: Fixed { raw: 0 } },
        inv_mass: Fixed { raw: 4294967296 },
        inv_principal_inertia: Fixed { raw: 25769803800 },
    };
    const RIGHT: MassProperties = MassProperties {
        local_com: Vec2 { x: Fixed { raw: 4294967296 }, y: Fixed { raw: 0 } },
        inv_mass: Fixed { raw: 4294967296 },
        inv_principal_inertia: Fixed { raw: 25769803800 },
    };
    /// Centre (1, 1), mass 2, inertia 8.
    const BODY: MassProperties = MassProperties {
        local_com: Vec2 { x: Fixed { raw: 4294967296 }, y: Fixed { raw: 4294967296 } },
        inv_mass: Fixed { raw: 2147483648 },
        inv_principal_inertia: Fixed { raw: 536870912 },
    };

    fn unit_cuboid_at(x: i32) -> MassProperties {
        let p = MassPropertiesTrait::from_cuboid(ONE, v(HALF, HALF));
        p.transform_by(Pose2Trait::new(v(i(x), ZERO), Rot2Trait::IDENTITY))
    }

    /// The kernels (before the inversion that `new` applies) against the golden mass and inertia:
    /// the 16-ulp tolerance of `tools/golden/README.md`, with the error printed for the report.
    #[test]
    fn test_kernels_against_golden() {
        let mut index = 0_u32;
        for c in cases() {
            let density = Fixed { raw: *c.density };
            let (mass, inertia) = match *c.shape {
                ShapeRaw::Ball(r) => ball_mass_inertia(density, Fixed { raw: r }),
                ShapeRaw::Cuboid(h) => cuboid_mass_inertia(
                    density, v(Fixed { raw: h.x }, Fixed { raw: h.y }),
                ),
                ShapeRaw::Capsule(k) => capsule_mass_inertia(
                    density,
                    norm2(Fixed { raw: k.b.x - k.a.x }, Fixed { raw: k.b.y - k.a.y }),
                    Fixed { raw: k.radius },
                ),
                _ => { continue; },
            };
            let (dm, di) = (
                abs_diff(mass.raw, *c.expected.mass),
                abs_diff(inertia.raw, *c.expected.principal_inertia),
            );
            println!("kernel case {}: mass {} ulp, inertia {} ulp", index, dm, di);
            assert!(dm <= 16 && di <= 16, "case {}", index);
            index += 1;
        }
        assert_eq!(index, 11);
    }

    #[test]
    fn test_probe_constants() {
        assert_eq!((unit_cuboid_at(-1), unit_cuboid_at(1)), (LEFT, RIGHT));
        assert_eq!(MassPropertiesTrait::new(v(ONE, ONE), TWO, i(8)), BODY);
    }

    #[test]
    fn test_new_inverts_and_zero_is_infinite() {
        let p = MassPropertiesTrait::new(v(ONE, TWO), i(4), i(8));
        assert_eq!(
            (p.inv_mass, p.inv_principal_inertia),
            (FixedTrait::from_ratio(1, 4), FixedTrait::from_ratio(1, 8)),
        );
        assert_eq!((p.mass(), p.principal_inertia()), (i(4), i(8)));
        // A zero mass or inertia is an infinite one: `inv(0) = 0`, and it round-trips to 0.
        let fixed_body = MassPropertiesTrait::new(v(ONE, TWO), ZERO, ZERO);
        assert_eq!((fixed_body.inv_mass, fixed_body.inv_principal_inertia), (ZERO, ZERO));
        assert_eq!((fixed_body.mass(), fixed_body.principal_inertia()), (ZERO, ZERO));
        assert!(MassPropertiesTrait::from_segment().is_zero());
        assert!(!p.is_zero());
    }

    #[test]
    fn test_shape_formulas_table() {
        // (density, half extents): mass = 4 hx hy density is exact, I = mass (hx^2 + hy^2) / 3.
        let cases: Span<(Fixed, Vec2, i32, i32)> = array![
            (ONE, v(HALF, HALF), 1, 6), (TWO, v(ONE, ONE), 8, 12), (ONE, v(i(3), HALF), 6, 6),
        ]
            .span();
        for (density, he, mass, unit_inertia_times_mass) in cases {
            let (m, inertia) = cuboid_mass_inertia(*density, *he);
            assert_eq!(m, i(*mass));
            // I = mass * (hx^2 + hy^2) / 3, hence * 3 / mass is the sum of squares.
            let sq = (*he.x * *he.x + *he.y * *he.y) * i(*mass) / i(3);
            near(inertia, sq, 2);
            assert_eq!(i(*unit_inertia_times_mass) / i(*unit_inertia_times_mass), ONE);
        }
        // Ball r = 1, density 1: mass pi, inertia pi / 2. Capsule of length 0 is that ball.
        let (m, inertia) = ball_mass_inertia(ONE, ONE);
        near(m, FixedTrait::from_raw(13493037705), 1);
        near(inertia, FixedTrait::from_raw(6746518852), 2);
        let (m2, inertia2) = capsule_mass_inertia(ONE, ZERO, ONE);
        assert_eq!((m2, inertia2), (m, inertia));
    }

    #[test]
    fn test_degenerate_shapes() {
        // Zero radius / extent / density: infinite mass in the inverse representation.
        for p in array![
            MassPropertiesTrait::from_ball(ONE, ZERO), MassPropertiesTrait::from_ball(ZERO, ONE),
            MassPropertiesTrait::from_cuboid(ONE, v(ZERO, ONE)),
            MassPropertiesTrait::from_capsule(ONE, v(ONE, ONE), v(ONE, ONE), ZERO),
        ]
            .span() {
            assert_eq!((*p.inv_mass, *p.inv_principal_inertia), (ZERO, ZERO));
        }
        // A zero-length capsule sits on its (shifted) centre.
        let c = MassPropertiesTrait::from_capsule(ONE, v(ONE, TWO), v(ONE, TWO), HALF);
        assert_eq!(c.local_com, v(ONE, TWO));
        assert_eq!(c.inv_mass, MassPropertiesTrait::from_ball(ONE, HALF).inv_mass);
    }

    #[test]
    fn test_add_sub_transform() {
        let (left, right) = (unit_cuboid_at(-1), unit_cuboid_at(1));
        assert_eq!(left.local_com, v(-ONE, ZERO));
        assert_eq!(
            (left.inv_mass, left.inv_principal_inertia),
            (ONE, MassPropertiesTrait::from_cuboid(ONE, v(HALF, HALF)).inv_principal_inertia),
        );
        let both = left + right;
        // Two unit-mass boxes of inertia 1/6, one unit either side of the origin: mass 2,
        // I = 2 (1/6 + 1) = 7/3, centre of mass at the origin.
        assert_eq!(both.local_com, v(ZERO, ZERO));
        near(both.mass(), TWO, 2);
        near(both.principal_inertia(), FixedTrait::from_ratio(7, 3), 8);
        // The sum is commutative and `Sub` undoes it.
        assert_eq!(right + left, both);
        let back = both - right;
        near(back.mass(), ONE, 8);
        near(back.local_com.x, -ONE, 8);
        near(back.local_com.y, ZERO, 2);
        near(back.principal_inertia(), left.principal_inertia(), 16);
        // Zero is neutral for `Add`; `Sub` returns `self` for a zero operand; a zero minuend stays
        // zero.
        let zero: MassProperties = Zero::zero();
        assert_eq!((left + zero, zero + left, left - zero, zero - left), (left, left, left, zero));
        // Subtracting the whole body leaves nothing (masses below epsilon clamp to zero).
        let nothing = left - left;
        assert_eq!((nothing.inv_mass, nothing.inv_principal_inertia), (ZERO, ZERO));
    }

    #[test]
    fn test_set_mass_and_inertia_helpers() {
        let base = MassPropertiesTrait::new(v(ONE, ONE), TWO, i(8));
        let mut heavier = base;
        heavier.set_mass(i(4), true);
        assert_eq!((heavier.mass(), heavier.principal_inertia()), (i(4), i(16)));
        let mut plain = base;
        plain.set_mass(i(4), false);
        assert_eq!((plain.mass(), plain.principal_inertia()), (i(4), i(8)));
        let mut infinite = base;
        infinite.set_mass(ZERO, true);
        assert_eq!(infinite.inv_mass, ZERO);
        assert_eq!(base.with_inertia(i(4)).principal_inertia(), i(4));
        assert_eq!(base.with_inertia(ZERO).inv_principal_inertia, ZERO);
        assert_eq!(base.with_inertia_scaled(TWO).principal_inertia(), i(16));
        assert_eq!(base.with_inertia_scaled(ZERO).inv_principal_inertia, ZERO);
        assert_eq!(base.with_inertia(i(4)).local_com, base.local_com);
    }

    #[test]
    fn test_transform_by_and_world_com() {
        let p = MassPropertiesTrait::new(v(ONE, ZERO), TWO, i(8));
        // Quarter turn then translation (0, 3): (1, 0) -> (0, 4); inverse masses are untouched.
        let pose = Pose2Trait::new(v(ZERO, i(3)), Rot2Trait::from_cos_sin(ZERO, ONE));
        let moved = p.transform_by(pose);
        assert_eq!(moved.local_com, v(ZERO, i(4)));
        assert_eq!(
            (moved.inv_mass, moved.inv_principal_inertia), (p.inv_mass, p.inv_principal_inertia),
        );
        assert_eq!(p.world_com(pose), moved.local_com);
    }

    /// Fused kernels against the composed-ops oracles. The absolute floor is the number of rounded
    /// product/division sites in the composed oracle; the relative term covers those sites after
    /// they are multiplied into the final value.
    #[test]
    #[fuzzer(runs: 96, seed: 20260920)]
    fn fuzz_kernels_against_composed(r: u16, hx: u16, hy: u16, len: u16, d: u8) {
        let density = Fixed { raw: (d.into() + 1) * 0x4000000 };
        let radius = Fixed { raw: (r.into() + 1) * 0x100000 };
        let he = v(
            Fixed { raw: (hx.into() + 1) * 0x100000 }, Fixed { raw: (hy.into() + 1) * 0x100000 },
        );
        let length = Fixed { raw: len.into() * 0x100000 };
        let (m, inertia) = ball_mass_inertia(density, radius);
        let (m2, inertia2) = alternatives::ball_mass_inertia(density, radius);
        near(m, m2, oracle_tolerance(m, 3));
        near(inertia, inertia2, oracle_tolerance(inertia, 6));
        let (m, inertia) = cuboid_mass_inertia(density, he);
        let (m2, inertia2) = alternatives::cuboid_mass_inertia(density, he);
        near(m, m2, oracle_tolerance(m, 2));
        near(inertia, inertia2, oracle_tolerance(inertia, 7));
        let (m, inertia) = capsule_mass_inertia(density, length, radius);
        let (m2, inertia2) = alternatives::capsule_mass_inertia(density, length, radius);
        near(m, m2, oracle_tolerance(m, 4));
        near(inertia, inertia2, oracle_tolerance(inertia, 20));
    }

    #[test]
    fn test_capsule_composed_counterexample_small_magnitude() {
        let density = Fixed { raw: 202 * 0x4000000 };
        let radius = Fixed { raw: 107 * 0x100000 };
        let length = Fixed { raw: 119 * 0x100000 };
        let (_m, inertia) = capsule_mass_inertia(density, length, radius);
        let (_m2, inertia2) = alternatives::capsule_mass_inertia(density, length, radius);
        assert_eq!(inertia.raw, 30447);
        assert_eq!(inertia2.raw, 30441);
        near(inertia, inertia2, oracle_tolerance(inertia, 20));
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_new() {
        let _ = MassPropertiesTrait::new(opaque(v(ONE, TWO)), opaque(i(4)), opaque(i(8)));
    }
    #[test]
    fn gas_ball_fused() {
        let _ = ball_mass_inertia(opaque(ONE), opaque(HALF));
    }
    #[test]
    fn gas_ball_composed() {
        let _ = alternatives::ball_mass_inertia(opaque(ONE), opaque(HALF));
    }
    #[test]
    fn gas_cuboid_fused() {
        let _ = cuboid_mass_inertia(opaque(ONE), opaque(v(HALF, ONE)));
    }
    #[test]
    fn gas_cuboid_composed() {
        let _ = alternatives::cuboid_mass_inertia(opaque(ONE), opaque(v(HALF, ONE)));
    }
    #[test]
    fn gas_capsule_fused() {
        let _ = capsule_mass_inertia(opaque(ONE), opaque(TWO), opaque(HALF));
    }
    #[test]
    fn gas_capsule_composed() {
        let _ = alternatives::capsule_mass_inertia(opaque(ONE), opaque(TWO), opaque(HALF));
    }
    #[test]
    fn gas_from_ball() {
        let _ = MassPropertiesTrait::from_ball(opaque(ONE), opaque(HALF));
    }
    #[test]
    fn gas_from_cuboid() {
        let _ = MassPropertiesTrait::from_cuboid(opaque(ONE), opaque(v(HALF, ONE)));
    }
    #[test]
    fn gas_from_capsule() {
        let _ = MassPropertiesTrait::from_capsule(
            opaque(ONE), opaque(v(ZERO, -ONE)), opaque(v(ZERO, ONE)), opaque(HALF),
        );
    }
    #[test]
    fn gas_mass_and_inertia() {
        let p = opaque(BODY);
        let _ = (p.mass(), p.principal_inertia());
    }
    #[test]
    fn gas_transform_by() {
        let pose = Pose2Trait::new(opaque(v(ZERO, i(3))), opaque(Rot2 { re: ZERO, im: ONE }));
        let _ = opaque(BODY).transform_by(pose);
    }
    #[test]
    fn gas_add() {
        let _ = opaque(LEFT) + opaque(RIGHT);
    }
    #[test]
    fn gas_sub() {
        let _ = opaque(BODY) - opaque(RIGHT);
    }
    #[test]
    fn gas_set_mass() {
        let mut p = opaque(BODY);
        p.set_mass(opaque(i(4)), true);
    }
    #[test]
    fn test_assign_ops_and_world_inv_inertia() {
        let mut p = LEFT;
        p += RIGHT;
        assert_eq!(p, LEFT + RIGHT);
        p -= RIGHT;
        assert_eq!(p, LEFT + RIGHT - RIGHT);
        let turned = Rot2 { re: ZERO, im: ONE };
        assert_eq!(BODY.world_inv_inertia(turned), BODY.inv_principal_inertia);
    }
    #[test]
    fn gas_add_assign() {
        let mut p = opaque(LEFT);
        p += opaque(RIGHT);
    }
}

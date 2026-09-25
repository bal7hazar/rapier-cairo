//! The vector-valued components of a collider (upstream `geometry/collider_components.rs`):
//! the parent link, the world pose and the way the mass is specified.

use fixed::{Fixed, ONE, ZERO};
use rapier_core::Handle;
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::{IDENTITY, Pose2};

/// The rigid-body a collider is attached to and where it sits on it.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderParent {
    /// Handle of the rigid-body this collider is attached to.
    pub handle: Handle,
    /// Constant pose of the collider relative to its parent body.
    pub pos_wrt_parent: Pose2,
}

/// World pose of a collider. Default: the identity.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderPosition {
    pub pose: Pose2,
}

pub impl ColliderPositionDefault of Default<ColliderPosition> {
    #[inline(always)]
    fn default() -> ColliderPosition {
        ColliderPosition { pose: IDENTITY }
    }
}

pub impl Pose2IntoColliderPosition of Into<Pose2, ColliderPosition> {
    #[inline(always)]
    fn into(self: Pose2) -> ColliderPosition {
        ColliderPosition { pose: self }
    }
}

/// How the mass of a collider is specified; the local [`MassProperties`] are resolved from it
/// and the shape by [`ColliderMassPropsTrait::mass_properties`].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ColliderMassProps {
    /// A uniform density: mass and inertia follow from the shape.
    Density: Fixed,
    /// A mass: the inertia follows from the shape (scaled with the mass).
    Mass: Fixed,
    /// Explicit mass properties, used as given.
    MassProperties: MassProperties,
}

/// Upstream default: a unit density.
pub impl ColliderMassPropsDefault of Default<ColliderMassProps> {
    #[inline(always)]
    fn default() -> ColliderMassProps {
        ColliderMassProps::Density(ONE)
    }
}

pub impl MassPropertiesIntoColliderMassProps of Into<MassProperties, ColliderMassProps> {
    #[inline(always)]
    fn into(self: MassProperties) -> ColliderMassProps {
        ColliderMassProps::MassProperties(self)
    }
}

#[generate_trait]
pub impl ColliderMassPropsImpl of ColliderMassPropsTrait {
    /// The local mass properties of a collider of shape `shape` (upstream
    /// `ColliderMassProps::mass_properties`).
    ///
    /// * `Density(d)`: `shape.mass_properties(d)`, one fused kernel; a zero density gives zero
    ///   (infinite-mass) properties.
    /// * `Mass(m)`: the properties of unit density with the mass replaced by `m` and the inertia
    ///   scaled along (`set_mass(m, true)`); a zero mass gives zero properties.
    /// * `MassProperties(p)`: `p` as is.
    /// #### Panics
    /// * `'Fixed: overflow'` when the mass or the inertia leaves the scalar range, see
    ///   [`MassPropertiesTrait`].
    fn mass_properties(self: ColliderMassProps, shape: Shape) -> MassProperties {
        match self {
            ColliderMassProps::Density(density) => {
                if density != ZERO {
                    shape.mass_properties(density)
                } else {
                    Default::default()
                }
            },
            ColliderMassProps::Mass(mass) => mass_variant(mass, shape),
            ColliderMassProps::MassProperties(mprops) => mprops,
        }
    }
}

/// The `Mass` variant: the properties of unit density with the mass replaced. Kept out of line on
/// purpose: inlined into the `match` of `mass_properties`, its two kernels are charged to every
/// arm (`Density` measured 63.0k gas instead of 34.7k, see `alternatives`).
#[inline(never)]
fn mass_variant(mass: Fixed, shape: Shape) -> MassProperties {
    if mass != ZERO {
        let mut mprops = shape.mass_properties(ONE);
        mprops.set_mass(mass, true);
        mprops
    } else {
        Default::default()
    }
}

#[cfg(test)]
mod alternatives {
    use fixed::{Fixed, ONE, ZERO};
    use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
    use rapier_geometry2d::shape::{Shape, ShapeTrait};
    use super::ColliderMassProps;

    /// Rejected storage: two words per collider instead of one boxed optional value.
    pub fn optional_box(
        config: Option<super::OneWayPlatform>,
    ) -> Option<Box<super::OneWayPlatform>> {
        config.map(|value| BoxTrait::new(value))
    }

    /// One `match` with every arm inline. Measured 63.0k gas on every path, against 34.7k
    /// (`Density`), 64.0k (`Mass`) and 18.7k (`MassProperties`) for the winner.
    pub fn mass_properties_inline(props: ColliderMassProps, shape: Shape) -> MassProperties {
        match props {
            ColliderMassProps::Density(density) => {
                if density != ZERO {
                    shape.mass_properties(density)
                } else {
                    Default::default()
                }
            },
            ColliderMassProps::Mass(mass) => {
                if mass != ZERO {
                    let mut mprops = shape.mass_properties(ONE);
                    mprops.set_mass(mass, true);
                    mprops
                } else {
                    Default::default()
                }
            },
            ColliderMassProps::MassProperties(mprops) => mprops,
        }
    }

    /// Both computing arms out of line: the extra call makes the `Density` path dearer (53.6k).
    pub fn mass_properties_split(props: ColliderMassProps, shape: Shape) -> MassProperties {
        match props {
            ColliderMassProps::Density(density) => density_variant(density, shape),
            ColliderMassProps::Mass(mass) => super::mass_variant(mass, shape),
            ColliderMassProps::MassProperties(mprops) => mprops,
        }
    }

    #[inline(never)]
    fn density_variant(density: Fixed, shape: Shape) -> MassProperties {
        if density != ZERO {
            shape.mass_properties(density)
        } else {
            Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
    use rapier_geometry2d::shape::{
        BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape, ShapeTrait,
    };
    use rapier_math::inv;
    use rapier_math::pose2::{IDENTITY, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::alternatives::{mass_properties_inline, mass_properties_split};
    use super::{
        ColliderMassProps, ColliderMassPropsTrait, ColliderPosition, Pose2IntoColliderPosition,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn near(a: Fixed, b: Fixed, ulps: i64) {
        let d = if a.raw > b.raw {
            a.raw - b.raw
        } else {
            b.raw - a.raw
        };
        assert!(d <= ulps, "{} vs {}", a.raw, b.raw);
    }

    fn shapes() -> Span<Shape> {
        array![
            Shape::Ball(BallTrait::new(ONE)), Shape::Cuboid(CuboidTrait::new(v(TWO, ONE))),
            Shape::Capsule(CapsuleTrait::new_y(HALF, HALF)),
            Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE))),
            Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))),
        ]
            .span()
    }

    #[test]
    fn gas_one_way_storage_boxed_option() {
        let value: Box<Option<super::OneWayPlatform>> = BoxTrait::new(opaque(None));
        let values = array![opaque(value), opaque(value), opaque(value), opaque(value)];
        assert!((*values.at(3)).unbox().is_none());
    }
    #[test]
    fn gas_one_way_storage_optional_box() {
        let value = super::alternatives::optional_box(opaque(None));
        let values = array![opaque(value), opaque(value), opaque(value), opaque(value)];
        assert!((*values.at(3)).is_none());
    }

    #[test]
    fn test_defaults_and_conversions() {
        let props: ColliderMassProps = Default::default();
        assert_eq!(props, ColliderMassProps::Density(ONE));
        let position: ColliderPosition = Default::default();
        assert_eq!(position.pose, IDENTITY);
        let pose = Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE });
        let converted: ColliderPosition = pose.into();
        assert_eq!(converted, ColliderPosition { pose });
        let explicit: MassProperties = Default::default();
        let converted: ColliderMassProps = explicit.into();
        assert_eq!(converted, ColliderMassProps::MassProperties(explicit));
    }

    /// Density resolves to the shape kernel, a zero density and a zero mass to the zero
    /// (infinite-mass) properties, explicit properties to themselves.
    #[test]
    fn test_density_and_explicit_variants() {
        let zero: MassProperties = Default::default();
        let custom = MassPropertiesTrait::new(v(HALF, ZERO), TWO, ONE);
        for shape in shapes() {
            let shape = *shape;
            assert_eq!(
                ColliderMassProps::Density(TWO).mass_properties(shape), shape.mass_properties(TWO),
            );
            assert_eq!(ColliderMassProps::Density(ZERO).mass_properties(shape), zero);
            assert_eq!(ColliderMassProps::Mass(ZERO).mass_properties(shape), zero);
            assert_eq!(ColliderMassProps::MassProperties(custom).mass_properties(shape), custom);
        }
    }

    /// `Mass(m)` keeps the centre of mass of the unit-density properties, sets the mass to `m`
    /// and scales the inertia with it: the inertia of a unit ball is `m / 2`, of the
    /// `4 x 2` box `m * 5 / 3`.
    #[test]
    fn test_mass_variant_scales_the_inertia() {
        let three = FixedTrait::from_int(3);
        let ball = ColliderMassProps::Mass(three).mass_properties(shapes().at(0).clone());
        near(ball.mass(), three, 4);
        near(ball.principal_inertia(), three / TWO, 64);
        assert_eq!(ball.local_com, v(ZERO, ZERO));
        let cuboid = ColliderMassProps::Mass(three).mass_properties(shapes().at(1).clone());
        near(cuboid.mass(), three, 4);
        near(cuboid.principal_inertia(), FixedTrait::from_int(5), 64);
        // A shape without area: the mass is set, the inertia stays infinite.
        let segment = ColliderMassProps::Mass(three).mass_properties(shapes().at(3).clone());
        assert_eq!(segment.inv_mass, inv(three));
        assert_eq!(segment.inv_principal_inertia, ZERO);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_density_ball() {
        let props = opaque(ColliderMassProps::Density(opaque(TWO)));
        let mprops = props.mass_properties(opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }

    #[test]
    fn gas_density_cuboid() {
        let props = opaque(ColliderMassProps::Density(opaque(TWO)));
        let mprops = props.mass_properties(opaque(Shape::Cuboid(CuboidTrait::new(v(TWO, ONE)))));
        assert!(mprops.inv_mass != ZERO);
    }

    #[test]
    fn gas_mass_ball() {
        let props = opaque(ColliderMassProps::Mass(opaque(TWO)));
        let mprops = props.mass_properties(opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }

    #[test]
    fn gas_explicit() {
        let props = opaque(
            ColliderMassProps::MassProperties(MassPropertiesTrait::new(v(ONE, ONE), TWO, ONE)),
        );
        let mprops = props.mass_properties(opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }

    /// The candidates agree with the winner on every variant and shape.
    #[test]
    fn test_alternatives_agree() {
        let custom = MassPropertiesTrait::new(v(HALF, ZERO), TWO, ONE);
        let variants = array![
            ColliderMassProps::Density(TWO), ColliderMassProps::Density(ZERO),
            ColliderMassProps::Mass(TWO), ColliderMassProps::Mass(ZERO),
            ColliderMassProps::MassProperties(custom),
        ];
        for props in variants.span() {
            for shape in shapes() {
                let expected = (*props).mass_properties(*shape);
                assert_eq!(mass_properties_inline(*props, *shape), expected);
                assert_eq!(mass_properties_split(*props, *shape), expected);
            }
        }
    }

    #[test]
    fn gas_density_ball_inline() {
        let props = opaque(ColliderMassProps::Density(opaque(TWO)));
        let mprops = mass_properties_inline(props, opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }

    #[test]
    fn gas_density_ball_split() {
        let props = opaque(ColliderMassProps::Density(opaque(TWO)));
        let mprops = mass_properties_split(props, opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }

    #[test]
    fn gas_mass_ball_inline() {
        let props = opaque(ColliderMassProps::Mass(opaque(TWO)));
        let mprops = mass_properties_inline(props, opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }

    #[test]
    fn gas_mass_ball_split() {
        let props = opaque(ColliderMassProps::Mass(opaque(TWO)));
        let mprops = mass_properties_split(props, opaque(Shape::Ball(BallTrait::new(ONE))));
        assert!(mprops.inv_mass != ZERO);
    }
}

/// Built-in one-way platform cone, in the collider's local frame.
/// `local_up` must be unit and `cos_allowed_angle` must be in [-1, 1].
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct OneWayPlatform {
    pub local_up: glam::Vec2,
    pub cos_allowed_angle: fixed::Fixed,
}

/// Box serialization stores the cone value, independent of allocation identity.
pub impl BoxedOneWayPlatformSerde of Serde<Box<Option<OneWayPlatform>>> {
    fn serialize(self: @Box<Option<OneWayPlatform>>, ref output: Array<felt252>) {
        let value = (*self).unbox();
        value.serialize(ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<Option<OneWayPlatform>>> {
        Some(BoxTrait::new(Serde::<Option<OneWayPlatform>>::deserialize(ref serialized)?))
    }
}
/// Structural equality of boxed one-way configurations.
pub impl BoxedOneWayPlatformPartialEq of PartialEq<Box<Option<OneWayPlatform>>> {
    fn eq(lhs: @Box<Option<OneWayPlatform>>, rhs: @Box<Option<OneWayPlatform>>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<Option<OneWayPlatform>>, rhs: @Box<Option<OneWayPlatform>>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

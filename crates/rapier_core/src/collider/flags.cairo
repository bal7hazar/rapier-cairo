//! The filtering / event flags of a collider (upstream `ColliderFlags`) and its scalar
//! companions `ColliderType` and `ColliderEnabled`.

use crate::interaction_groups::{InteractionGroups, InteractionGroupsTrait};
use super::active_collision_types::ActiveCollisionTypes;
use super::events::ActiveEvents;
use super::hooks::ActiveHooks;

/// The type of a collider.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ColliderType {
    /// A collider that generates contacts and contact events.
    Solid,
    /// A collider that generates intersections and intersection events.
    Sensor,
}

/// Predicates of [`ColliderType`].
#[generate_trait]
pub impl ColliderTypeImpl of ColliderTypeTrait {
    /// Is this collider a sensor?
    #[inline(always)]
    fn is_sensor(self: ColliderType) -> bool {
        match self {
            ColliderType::Sensor => true,
            ColliderType::Solid => false,
        }
    }
}

/// Whether a collider is enabled.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub enum ColliderEnabled {
    /// The collider is enabled.
    #[default]
    Enabled,
    /// The collider was not disabled explicitly but its rigid-body is disabled.
    DisabledByParent,
    /// The collider is disabled explicitly.
    Disabled,
}

/// Flags controlling filtering, modification and events of a collider.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderFlags {
    /// Which body-type combinations this collider can collide with. Default: dynamic - dynamic,
    /// dynamic - kinematic, dynamic - fixed.
    pub active_collision_types: ActiveCollisionTypes,
    /// Groups controlling which pairs of colliders can interact (generate events or contacts).
    /// Default: [`InteractionGroupsTrait::all`].
    pub collision_groups: InteractionGroups,
    /// Groups controlling which pairs of colliders have their contact points taken into account
    /// for force computation. Default: [`InteractionGroupsTrait::all`].
    pub solver_groups: InteractionGroups,
    /// Physics hooks enabled for pairs involving this collider. Default: none. Plain data: hooks
    /// are cut from the port.
    pub active_hooks: ActiveHooks,
    /// Events enabled for this collider. Default: none.
    pub active_events: ActiveEvents,
    /// Whether the collider is enabled. Default: `Enabled`.
    pub enabled: ColliderEnabled,
}

/// Upstream default: default collision types, `all()` groups, no hooks, no events, enabled.
pub impl ColliderFlagsDefault of Default<ColliderFlags> {
    #[inline(always)]
    fn default() -> ColliderFlags {
        ColliderFlags {
            active_collision_types: Default::default(),
            collision_groups: InteractionGroupsTrait::all(),
            solver_groups: InteractionGroupsTrait::all(),
            active_hooks: Default::default(),
            active_events: Default::default(),
            enabled: ColliderEnabled::Enabled,
        }
    }
}

/// The default flags with the given hooks (upstream `From<ActiveHooks> for ColliderFlags`).
pub impl ActiveHooksIntoColliderFlags of Into<ActiveHooks, ColliderFlags> {
    #[inline(always)]
    fn into(self: ActiveHooks) -> ColliderFlags {
        ColliderFlags { active_hooks: self, ..Default::default() }
    }
}

/// The default flags with the given events (upstream `From<ActiveEvents> for ColliderFlags`).
pub impl ActiveEventsIntoColliderFlags of Into<ActiveEvents, ColliderFlags> {
    #[inline(always)]
    fn into(self: ActiveEvents) -> ColliderFlags {
        ColliderFlags { active_events: self, ..Default::default() }
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use crate::interaction_groups::{ALL, GROUP_1, InteractionGroupsTrait, InteractionTestMode};
    use super::super::active_collision_types::{DYNAMIC_DYNAMIC, DYNAMIC_FIXED, DYNAMIC_KINEMATIC};
    use super::super::events::{ActiveEventsTrait, COLLISION_EVENTS};
    use super::super::hooks::{ActiveHooksTrait, MODIFY_SOLVER_CONTACTS};
    use super::{ColliderEnabled, ColliderFlags, ColliderType, ColliderTypeTrait};

    #[test]
    fn test_collider_type() {
        assert!(ColliderType::Sensor.is_sensor());
        assert!(!ColliderType::Solid.is_sensor());
    }

    #[test]
    fn test_enabled_default() {
        let enabled: ColliderEnabled = Default::default();
        assert_eq!(enabled, ColliderEnabled::Enabled);
    }

    #[test]
    fn test_flags_default() {
        let flags: ColliderFlags = Default::default();
        assert_eq!(
            flags.active_collision_types, DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED,
        );
        // `InteractionGroups::all()`, not `InteractionGroups::default()` (which is GROUP_1).
        assert_eq!(flags.collision_groups, InteractionGroupsTrait::all());
        assert_eq!(flags.solver_groups, InteractionGroupsTrait::all());
        assert_eq!(flags.collision_groups.memberships, ALL);
        assert_eq!(flags.collision_groups.filter, ALL);
        assert_eq!(flags.collision_groups.test_mode, InteractionTestMode::And);
        assert!(flags.collision_groups.memberships != GROUP_1);
        assert_eq!(flags.active_hooks, ActiveHooksTrait::empty());
        assert_eq!(flags.active_events, ActiveEventsTrait::empty());
        assert_eq!(flags.enabled, ColliderEnabled::Enabled);
    }

    #[test]
    fn test_flags_from_hooks_and_events() {
        let default: ColliderFlags = Default::default();
        let from_hooks: ColliderFlags = MODIFY_SOLVER_CONTACTS.into();
        assert_eq!(from_hooks, ColliderFlags { active_hooks: MODIFY_SOLVER_CONTACTS, ..default });
        let from_events: ColliderFlags = COLLISION_EVENTS.into();
        assert_eq!(from_events, ColliderFlags { active_events: COLLISION_EVENTS, ..default });
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_default() {
        let flags: ColliderFlags = Default::default();
        assert!(opaque(flags).enabled == ColliderEnabled::Enabled);
    }

    #[test]
    fn gas_is_sensor() {
        assert!(opaque(ColliderType::Sensor).is_sensor());
    }

    #[test]
    fn gas_from_events() {
        let flags: ColliderFlags = opaque(COLLISION_EVENTS).into();
        assert!(flags.active_events.bits == 1);
    }
}

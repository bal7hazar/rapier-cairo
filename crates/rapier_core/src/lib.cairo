//! Dimension-agnostic data structures shared by every rapier.cairo crate.

pub mod data;
pub mod interaction_groups;

pub use data::arena::{Arena, ArenaState, ArenaStateTrait, ArenaTrait};
pub use data::handle::{Handle, HandleTrait, INVALID_HANDLE};
pub use data::union_find::{UnionFind, UnionFindTrait};
pub use interaction_groups::{
    Group, GroupTrait, InteractionGroups, InteractionGroupsTrait, InteractionTestMode,
};

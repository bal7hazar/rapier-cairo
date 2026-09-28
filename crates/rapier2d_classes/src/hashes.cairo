//! The class hashes of the declared classes, as the caller's step compiles them.

use starknet::ClassHash;

/// Where the split step finds the declared classes it library-calls. A game declares the three
/// classes of this crate, then compiles its step with an impl of constants:
///
/// ```cairo
/// impl GameClasses of ClassHashes {
///     fn contact_ball() -> ClassHash {
///         const H: ClassHash = 0x..._felt252.try_into().unwrap();
///         H
///     }
///     ...
/// }
/// ```
///
/// A constant costs no Cairo step; reading the hash from storage costs ≈ 200 steps per call
/// (`tests/split.cairo`, `steps_split_stored_*`).
pub trait ClassHashes {
    /// The class hash of `ContactBallClass` (`crate::contact`).
    fn contact_ball() -> ClassHash;
    /// The class hash of `ContactPolygonClass` (`crate::contact`).
    fn contact_polygon() -> ClassHash;
    /// The class hash of `SolverClass` (`crate::solver`).
    fn solver() -> ClassHash;
}

/// Errors of the library calls.
pub mod errors {
    /// A class returned felts that do not decode as its result.
    pub const DECODE: felt252 = 'Classes: decode';
}

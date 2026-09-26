//! Starknet contract fixtures that link the `rapier2d` step into deployable classes, so that the
//! compiled class size of a game-like consumer is tracked against the network limits (work
//! package CS1).
//!
//! Measurement fixture, not a product (never published): `scripts/bytecode_size.py` builds this
//! package in the release profile and reports the Sierra and CASM sizes of every contract
//! (`gas/bytecode.size`, checked by CI's `bytecode` job); the analysis is in
//! `docs/research/class-size.md`. Every input comes from calldata, so that nothing is
//! constant-folded. The scenes live in `scene`, the contracts in `sink`.

pub mod scene;
pub mod sink;

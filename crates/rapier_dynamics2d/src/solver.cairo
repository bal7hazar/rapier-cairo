//! Scalar soft-contact solver, in caller-supplied pair-slot/manifold order.
//! DF owns external-force integration and the substep driver; see `contact` for stage order.
pub mod body;
pub mod contact;

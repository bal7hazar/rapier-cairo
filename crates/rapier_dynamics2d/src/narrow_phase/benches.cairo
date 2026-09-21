//! Gas probes of the narrow phase. Every probe of a scene pays the same setup (sets and broad
//! phase, `gas_setup_*`); `gas_step1_*` adds one `compute_contacts` on an empty narrow phase
//! (nothing to carry over), `gas_step2_*` a second one that carries the first step's pairs over.
//! Net costs: `step1 - setup` without carry-over, `step2 - step1` with it, per candidate
//! (`merge`: shipped sorted merge, `dict`: `alternatives::DictCarryOver`).
//!
//! Layouts: `stack` = `n` balls stacked on a halfspace (`2n - 1` pairs, `n` touching), `sparse`
//! = `n` balls side by side on it (`n` pairs, all touching).

use rapier_testing::opaque;
use super::alternatives::compute_contacts_dict;
use super::mock::{MockDispatcher, PREDICTION, broad_phase, scene};
use super::{NarrowPhase, NarrowPhaseTrait};

/// Builds the scene and runs `steps` narrow-phase steps with the chosen candidate.
#[inline(never)]
fn run(n: u32, stack: bool, steps: u32, dict: bool) {
    let (mut bodies, mut colliders) = scene(n, stack);
    let pairs = broad_phase(ref bodies, ref colliders, PREDICTION);
    let mut narrow_phase: NarrowPhase = NarrowPhaseTrait::new();
    let mut i = 0;
    while i != steps {
        let _ = if dict {
            compute_contacts_dict::<
                MockDispatcher,
            >(ref narrow_phase, PREDICTION, ref bodies, ref colliders, pairs.span())
        } else {
            narrow_phase
                .compute_contacts::<
                    MockDispatcher,
                >(PREDICTION, ref bodies, ref colliders, pairs.span())
        };
        i += 1;
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_setup_stack_2() {
    run(opaque(2), true, 0, false);
}

#[test]
fn gas_setup_stack_8() {
    run(opaque(8), true, 0, false);
}

#[test]
fn gas_setup_stack_32() {
    run(opaque(32), true, 0, false);
}

#[test]
fn gas_setup_sparse_8() {
    run(opaque(8), false, 0, false);
}

#[test]
fn gas_setup_sparse_32() {
    run(opaque(32), false, 0, false);
}

#[test]
fn gas_setup_sparse_2() {
    run(opaque(2), false, 0, false);
}

#[test]
fn gas_step1_merge_stack_2() {
    run(opaque(2), true, 1, false);
}

#[test]
fn gas_step1_merge_stack_8() {
    run(opaque(8), true, 1, false);
}

#[test]
fn gas_step1_merge_stack_32() {
    run(opaque(32), true, 1, false);
}

#[test]
fn gas_step1_merge_sparse_2() {
    run(opaque(2), false, 1, false);
}

#[test]
fn gas_step1_merge_sparse_8() {
    run(opaque(8), false, 1, false);
}

#[test]
fn gas_step1_merge_sparse_32() {
    run(opaque(32), false, 1, false);
}

#[test]
fn gas_step1_dict_stack_2() {
    run(opaque(2), true, 1, true);
}

#[test]
fn gas_step1_dict_stack_8() {
    run(opaque(8), true, 1, true);
}

#[test]
fn gas_step1_dict_stack_32() {
    run(opaque(32), true, 1, true);
}

#[test]
fn gas_step1_dict_sparse_2() {
    run(opaque(2), false, 1, true);
}

#[test]
fn gas_step1_dict_sparse_8() {
    run(opaque(8), false, 1, true);
}

#[test]
fn gas_step1_dict_sparse_32() {
    run(opaque(32), false, 1, true);
}

#[test]
fn gas_step2_merge_stack_2() {
    run(opaque(2), true, 2, false);
}

#[test]
fn gas_step2_merge_stack_8() {
    run(opaque(8), true, 2, false);
}

#[test]
fn gas_step2_merge_stack_32() {
    run(opaque(32), true, 2, false);
}

#[test]
fn gas_step2_merge_sparse_2() {
    run(opaque(2), false, 2, false);
}

#[test]
fn gas_step2_merge_sparse_8() {
    run(opaque(8), false, 2, false);
}

#[test]
fn gas_step2_merge_sparse_32() {
    run(opaque(32), false, 2, false);
}

#[test]
fn gas_step2_dict_stack_2() {
    run(opaque(2), true, 2, true);
}

#[test]
fn gas_step2_dict_stack_8() {
    run(opaque(8), true, 2, true);
}

#[test]
fn gas_step2_dict_stack_32() {
    run(opaque(32), true, 2, true);
}

#[test]
fn gas_step2_dict_sparse_2() {
    run(opaque(2), false, 2, true);
}

#[test]
fn gas_step2_dict_sparse_8() {
    run(opaque(8), false, 2, true);
}

#[test]
fn gas_step2_dict_sparse_32() {
    run(opaque(32), false, 2, true);
}

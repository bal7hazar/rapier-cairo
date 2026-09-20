//! G: methodology pitfalls (constant folding, inlining) and a few extra kernels.
use cubit::f64::{Fixed as CFixed, FixedTrait as CFixedTrait};
use numbench::bb::{bb, sink};
use numbench::reprs::felt::{Fe, check};
use numbench::reprs::i64b::{I64b, fused6};

fn a() -> I64b { I64b { v: bb(15032385536) } }
fn b() -> I64b { I64b { v: bb(-5368709120) } }

#[inline(always)]
fn mul_always(x: I64b, y: I64b) -> I64b { x * y }
#[inline(never)]
fn mul_never(x: I64b, y: I64b) -> I64b { x * y }

#[test]
fn base_b2() { let x = a(); let y = b(); let _ = y; sink(x); }
#[test]
fn base_f1() { let x = Fe { v: bb(15032385536) }; sink(x); }

#[test]
fn b2_mul_blackboxed_inputs() { let x = a(); let y = b(); sink(x * y); }
/// Inputs are literals: the compiler may fold the whole product at compile time.
#[test]
fn b2_mul_literal_inputs_no_blackbox() { let x = a(); let y = b(); let _ = x; let _ = y; sink(I64b { v: 15032385536 } * I64b { v: -5368709120 }); }
#[test]
fn b2_mul_x4_chain_default_inlining() { let x = a(); let y = b(); sink(x * y * y * y * y); }
#[test]
fn b2_mul_x4_chain_inline_always() { let x = a(); let y = b(); sink(mul_always(mul_always(mul_always(mul_always(x, y), y), y), y)); }
#[test]
fn b2_mul_x4_chain_inline_never() { let x = a(); let y = b(); sink(mul_never(mul_never(mul_never(mul_never(x, y), y), y), y)); }
#[test]
fn b2_dot6_fused_single_rescale() { let x = a(); let y = b(); sink(fused6([x, y, x, y, x, y], [y, y, x, x, y, x])); }
#[test]
fn b2_dot6_naive_6mul_5add() { let x = a(); let y = b(); sink(x * y + y * y + x * x + y * x + x * y + y * x); }
#[test]
fn f1_felt_storage_boundary_check() { let x = Fe { v: bb(15032385536) }; sink(check(x)); }

#[test]
fn check_dot6() {
    let x = I64b { v: 15032385536 }; let y = I64b { v: -5368709120 };
    let f = fused6([x, y, x, y, x, y], [y, y, x, x, y, x]);
    let n = x * y + y * y + x * x + y * x + x * y + y * x;
    let d = f.v - n.v;
    assert!(d < 8 && d > -8);
    // 4*(3.5*-1.25) + 1.5625 + 12.25 = -3.6875
    assert!(f.v == -15837691904);
}

/// cubit (and Orion, same code) sign-magnitude pitfall: 0 * -1 yields "negative zero".
#[test]
fn check_cubit_negative_zero() {
    let z: CFixed = CFixedTrait::new(bb(0), false) * CFixedTrait::new(bb(4294967296), true);
    println!("NEGZERO mag={} sign={} eq_zero={} lt_zero={}", z.mag, z.sign, z == CFixedTrait::ZERO(), z < CFixedTrait::ZERO());
}
/// 32.32 squared-length overflow: |v| = 50000 -> v.v = 2.5e9 > 2^31, but the wide sqrt still works.
#[test]
fn check_length_wide_no_overflow() {
    let x = I64b { v: bb(128849018880000) }; // 30000.0
    let y = I64b { v: bb(171798691840000) }; // 40000.0
    let l = numbench::reprs::i64b::length_wide2(x, y);
    assert!(l.v == 214748364800000); // 50000.0 exactly
}
#[test]
#[should_panic]
fn check_length_naive_overflows() {
    let x = I64b { v: bb(128849018880000) };
    let y = I64b { v: bb(171798691840000) };
    sink(x * x + y * y);
}

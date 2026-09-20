//! B (extra): mul-then-shift and division primitives.
use numbench::bb::{bb, sink};
use numbench::micro;

const A: u64 = 15032385536; // 3.5
const B: u64 = 5368709120; // 1.25

#[test]
fn base_u2() { let a = bb(A); let b = bb(B); let _ = b; sink(a); }
#[test]
fn base_u3() { let a = bb(A); let b = bb(B); let d = bb(0x100000000_u128); let _ = b; let _ = d; sink(a); }
#[test]
fn base_w1() { let a = bb(0x123456789abcdef0123456789_u128); sink(a); }
#[test]
fn base_w2() { let a = bb(0x123456789abcdef0123456789_u128); let d = bb(0x100000000_u128); let _ = d; sink(a); }
#[test]
fn base_x2() { let a = bb(0x123456789abcdef0123456789abcdef0123456789_u256); let d = bb(0x100000000_u256); let _ = d; sink(a); }
#[test]
fn base_y2() { let a = bb(0x123456789abcdef01_u128); let d = bb(0x100000000_u128); let _ = d; sink(u256 { low: a, high: 0 }); }

#[test]
fn u2_mulshift_div_operator_literal() { let a = bb(A); let b = bb(B); sink(micro::mulshift_div_operator(a, b)); }
#[test]
fn u2_mulshift_divrem_const_nonzero() { let a = bb(A); let b = bb(B); sink(micro::mulshift_divrem_const(a, b)); }
#[test]
fn u3_mulshift_div_runtime_divisor() { let a = bb(A); let b = bb(B); let d = bb(0x100000000_u128); sink(micro::mulshift_div_runtime(a, b, d)); }
#[test]
fn u2_mulshift_bounded_int() { let a = bb(A); let b = bb(B); sink(micro::mulshift_bounded(a, b)); }
#[test]
fn u2_mulshift_felt() { let a = bb(A); let b = bb(B); sink(micro::mulshift_felt(a, b)); }
#[test]
fn u2_u64_wide_mul_only() { let a = bb(A); let b = bb(B); sink(micro::u64_wide_mul_only(a, b)); }
#[test]
fn u2_u64_checked_mul() { let a = bb(3_u64); let b = bb(B); sink(micro::u64_checked_mul(a, b)); }
#[test]
fn u2_u64_div_const() { let a = bb(A); let b = bb(B); let _ = b; sink(micro::u64_div_const(a)); }
#[test]
fn u2_u64_div_runtime() { let a = bb(A); let b = bb(B); sink(micro::u64_div_runtime(a, b)); }
#[test]
fn w1_u128_div_const() { let a = bb(0x123456789abcdef0123456789_u128); sink(micro::u128_div_const(a)); }
#[test]
fn w2_u128_div_runtime() { let a = bb(0x123456789abcdef0123456789_u128); let d = bb(0x100000000_u128); sink(micro::u128_div_runtime(a, d)); }
#[test]
fn w2_u128_checked_mul() { let a = bb(0x123456789abcdef01_u128); let d = bb(0x100000000_u128); sink(micro::u128_checked_mul(a, d)); }
#[test]
fn y2_u128_wide_mul_only() { let a = bb(0x123456789abcdef01_u128); let d = bb(0x100000000_u128); sink(micro::u128_wide_mul_only(a, d)); }
#[test]
fn x2_u256_div_runtime() { let a = bb(0x123456789abcdef0123456789abcdef0123456789_u256); let d = bb(0x100000000_u256); sink(micro::u256_div_runtime(a, d)); }

#[test]
fn check_correctness() {
    let e: u64 = 18790481920; // 4.375
    assert!(micro::mulshift_div_operator(A, B) == e);
    assert!(micro::mulshift_divrem_const(A, B) == e);
    assert!(micro::mulshift_div_runtime(A, B, 0x100000000) == e);
    assert!(micro::mulshift_bounded(A, B) == e);
    assert!(micro::mulshift_felt(A, B) == e);
}

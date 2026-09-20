//! F: math vs bitwise vs loop.
use numbench::bb::{bb, sink};
use numbench::micro;

const X: u64 = 0x123456789abcdef1;

#[test]
fn base_p1() { let x = bb(X); sink((x, x)); }
#[test]
fn base_q1() { let x = bb(X); sink(x); }
#[test]
fn base_r1() { let x = bb(X); let _ = x; sink(false); }
#[test]
fn base_k2() { let x = bb(X); let k = bb(13_u32); let _ = k; sink(x); }
#[test]
fn base_h2() { let h = bb(0x12345678_u32); let l = bb(0x9abcdef1_u32); let _ = l; let w: u64 = h.into(); sink(w); }
#[test]
fn base_t1() { let x = bb(X); let _ = x; sink((1_u32, 2_u32)); }

#[test]
fn p1_split_math_divrem() { let x = bb(X); sink(micro::split_math(x)); }
#[test]
fn t1_split_math_bounded_int() { let x = bb(X); sink(micro::split_math_bounded(x)); }
#[test]
fn p1_split_bitwise_and() { let x = bb(X); sink(micro::split_bitwise(x)); }
#[test]
fn p1_split_loop_32_halvings() { let x = bb(X); sink(micro::split_loop(x)); }
#[test]
fn q1_low32_math_divrem() { let x = bb(X); sink(micro::split_math_lo_only(x)); }
#[test]
fn q1_low32_bitwise_and() { let x = bb(X); sink(micro::split_bitwise_lo_only(x)); }
#[test]
fn r1_parity_math_divrem() { let x = bb(X); sink(micro::parity_math(x)); }
#[test]
fn r1_parity_bitwise_and() { let x = bb(X); sink(micro::parity_bitwise(x)); }
#[test]
fn k2_shl13_math_table_mul() { let x = bb(0x1234_u64); let k = bb(13_u32); sink(micro::shl_math_table(x, k)); }
#[test]
fn k2_shl13_math_core_pow() { let x = bb(0x1234_u64); let k = bb(13_u32); sink(micro::shl_math_pow(x, k)); }
#[test]
fn k2_shl13_loop_doubling() { let x = bb(0x1234_u64); let k = bb(13_u32); sink(micro::shl_loop(x, k)); }
#[test]
fn h2_pack_math_checked() { let h = bb(0x12345678_u32); let l = bb(0x9abcdef1_u32); sink(micro::pack_math(h, l)); }
#[test]
fn h2_pack_math_bounded_int() { let h = bb(0x12345678_u32); let l = bb(0x9abcdef1_u32); sink(micro::pack_math_bounded(h, l)); }
#[test]
fn h2_pack_math_felt() { let h = bb(0x12345678_u32); let l = bb(0x9abcdef1_u32); sink(micro::pack_math_felt(h, l)); }
#[test]
fn h2_pack_bitwise_or() { let h = bb(0x12345678_u32); let l = bb(0x9abcdef1_u32); sink(micro::pack_bitwise(h, l)); }

#[test]
fn check_correctness() {
    assert!(micro::split_math(X) == (0x12345678, 0x9abcdef1));
    assert!(micro::split_math_bounded(X) == (0x12345678, 0x9abcdef1));
    assert!(micro::split_bitwise(X) == (0x12345678, 0x9abcdef1));
    assert!(micro::split_loop(X) == (0x12345678, 0x9abcdef1));
    assert!(micro::parity_math(X) && micro::parity_bitwise(X));
    assert!(micro::shl_math_table(0x1234, 13) == 0x1234 * 8192);
    assert!(micro::shl_math_pow(0x1234, 13) == 0x1234 * 8192);
    assert!(micro::shl_loop(0x1234, 13) == 0x1234 * 8192);
    assert!(micro::pack_math(0x12345678, 0x9abcdef1) == X);
    assert!(micro::pack_math_bounded(0x12345678, 0x9abcdef1) == X);
    assert!(micro::pack_math_felt(0x12345678, 0x9abcdef1) == X);
    assert!(micro::pack_bitwise(0x12345678, 0x9abcdef1) == X);
}

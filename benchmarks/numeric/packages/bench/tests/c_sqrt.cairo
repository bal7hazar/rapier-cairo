//! C: sqrt implementations on an unsigned 32.32 magnitude.
use numbench::bb::{bb, sink};
use numbench::sqrts;

const X: u64 = 5302428712241; // 1234.5678
const EXPECT: u64 = 150909767438; // floor(sqrt(X << 32)) = 35.13641...

#[test]
fn base_s1() { let x = bb(X); sink(x); }
#[test]
fn s1_sqrt_cubit_style_checked_mul() { let x = bb(X); sink(sqrts::sqrt_cubit_style(x)); }
#[test]
fn s1_sqrt_core_u128_widemul() { let x = bb(X); sink(sqrts::sqrt_core_u128(x)); }
#[test]
fn s1_sqrt_core_u64_lowprec() { let x = bb(X); sink(sqrts::sqrt_core_u64_lowprec(x)); }
#[test]
fn s1_sqrt_newton8_loop() { let x = bb(X); sink(sqrts::sqrt_newton_loop(x)); }
#[test]
fn s1_sqrt_newton8_unrolled() { let x = bb(X); sink(sqrts::sqrt_newton_unrolled(x)); }
#[test]
fn s1_inv_sqrt_core_then_div() { let x = bb(X); sink(sqrts::inv_sqrt_core(x)); }
#[test]
fn s1_inv_sqrt_div_then_core() { let x = bb(X); sink(sqrts::inv_sqrt_div_first(x)); }

#[test]
fn check_values() {
    println!("SQRT exact {}", sqrts::sqrt_core_u128(X));
    println!("SQRT cubit {}", sqrts::sqrt_cubit_style(X));
    println!("SQRT lowprec {}", sqrts::sqrt_core_u64_lowprec(X));
    println!("SQRT newton_loop {}", sqrts::sqrt_newton_loop(X));
    println!("SQRT newton_unrolled {}", sqrts::sqrt_newton_unrolled(X));
    println!("SQRT inv_a {}", sqrts::inv_sqrt_core(X));
    println!("SQRT inv_b {}", sqrts::inv_sqrt_div_first(X));
    // Newton with 8 fixed iterations on small / large inputs
    println!("SQRT newton_small {} exact {}", sqrts::sqrt_newton_loop(42950), sqrts::sqrt_core_u128(42950)); // 1e-5
    println!("SQRT newton_large {} exact {}", sqrts::sqrt_newton_loop(4294967296000000), sqrts::sqrt_core_u128(4294967296000000)); // 1e6
    assert!(sqrts::sqrt_core_u128(X) == EXPECT);
}

//! D: accuracy sweeps. Prints raw 32.32 results; scripts/sweep.py compares them with libm.
use cubit::f64::{Fixed, FixedTrait};
use cubit::f64::math::trig as ctrig;
use numbench::reprs::i64b::I64b;
use numbench::trig;

const N: u32 = 201;
const START: i64 = -30064771072; // -7.0
const STEP: i64 = 300647711; // 0.07 -> [-7, 7]

fn cubit_of(v: i64) -> Fixed {
    if v < 0 { FixedTrait::new((-v).try_into().unwrap(), true) } else { FixedTrait::new(v.try_into().unwrap(), false) }
}
fn pr(name: ByteArray, i: u32, x: i64, mag: u64, sign: bool) {
    println!("SWEEP {} {} {} {} {}", name, i, x, mag, if sign { 1_u8 } else { 0_u8 });
}
fn pri(name: ByteArray, i: u32, x: i64, r: I64b) {
    if r.v < 0 { pr(name, i, x, (-r.v).try_into().unwrap(), true) } else { pr(name, i, x, r.v.try_into().unwrap(), false) }
}

#[test]
fn sweep_cubit_sin() {
    for i in 0..N { let x = START + STEP * i.into(); let r = ctrig::sin(cubit_of(x)); pr("cubit_sin", i, x, r.mag, r.sign); }
}
#[test]
fn sweep_cubit_sin_fast() {
    for i in 0..N { let x = START + STEP * i.into(); let r = ctrig::sin_fast(cubit_of(x)); pr("cubit_sin_fast", i, x, r.mag, r.sign); }
}
#[test]
fn sweep_cubit_cos() {
    for i in 0..N { let x = START + STEP * i.into(); let r = ctrig::cos(cubit_of(x)); pr("cubit_cos", i, x, r.mag, r.sign); }
}
#[test]
fn sweep_cubit_cos_fast() {
    for i in 0..N { let x = START + STEP * i.into(); let r = ctrig::cos_fast(cubit_of(x)); pr("cubit_cos_fast", i, x, r.mag, r.sign); }
}
#[test]
fn sweep_cubit_atan() {
    for i in 0..N { let x = START + STEP * i.into(); let r = ctrig::atan(cubit_of(x)); pr("cubit_atan", i, x, r.mag, r.sign); }
}
#[test]
fn sweep_cubit_atan_fast() {
    for i in 0..N { let x = START + STEP * i.into(); let r = ctrig::atan_fast(cubit_of(x)); pr("cubit_atan_fast", i, x, r.mag, r.sign); }
}
#[test]
fn sweep_poly_sin5() {
    for i in 0..N { let x = START + STEP * i.into(); pri("poly_sin5", i, x, trig::sin_poly5(I64b { v: x })); }
}
#[test]
fn sweep_poly_sin7() {
    for i in 0..N { let x = START + STEP * i.into(); pri("poly_sin7", i, x, trig::sin_poly7(I64b { v: x })); }
}
#[test]
fn sweep_poly_sin9() {
    for i in 0..N { let x = START + STEP * i.into(); pri("poly_sin9", i, x, trig::sin_poly9(I64b { v: x })); }
}
#[test]
fn sweep_poly_cos7() {
    for i in 0..N { let x = START + STEP * i.into(); pri("poly_cos7", i, x, trig::cos_poly7(I64b { v: x })); }
}
#[test]
fn sweep_bhaskara_sin() {
    for i in 0..N { let x = START + STEP * i.into(); pri("bhaskara_sin", i, x, trig::sin_bhaskara(I64b { v: x })); }
}
#[test]
fn sweep_poly_atan() {
    // atan(x) = atan2(x, 1)
    for i in 0..N { let x = START + STEP * i.into(); pri("poly_atan", i, x, trig::atan2_poly11(I64b { v: x }, I64b { v: 4294967296 })); }
}

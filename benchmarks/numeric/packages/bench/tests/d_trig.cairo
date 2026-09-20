//! D: trig cost (angle = 0.7 rad and 2.5 rad) - cubit Taylor vs cubit LUT vs polynomial on i64.
use cubit::f64::{Fixed, FixedTrait};
use cubit::f64::math::trig as ctrig;
use numbench::bb::{bb, sink};
use numbench::reprs::i64b::I64b;
use numbench::trig;

fn ang() -> I64b { I64b { v: bb(3006477107) } } // 0.7
fn ang2() -> I64b { I64b { v: bb(10737418240) } } // 2.5
fn cang() -> Fixed { FixedTrait::new(bb(3006477107), false) }
fn cang2() -> Fixed { FixedTrait::new(bb(10737418240), false) }

#[test]
fn base_i1() { let x = ang(); sink(x); }
#[test]
fn base_i2() { let x = ang(); let y = ang2(); let _ = y; sink(x); }
#[test]
fn base_c1() { let x = cang(); sink(x); }
#[test]
fn base_c2() { let x = cang(); let y = cang2(); let _ = y; sink(x); }

#[test]
fn c1_cubit_sin_taylor_0p7() { let x = cang(); sink(ctrig::sin(x)); }
#[test]
fn c1_cubit_sin_taylor_2p5() { let x = cang2(); sink(ctrig::sin(x)); }
#[test]
fn c1_cubit_sin_fast_lut_0p7() { let x = cang(); sink(ctrig::sin_fast(x)); }
#[test]
fn c1_cubit_sin_fast_lut_2p5() { let x = cang2(); sink(ctrig::sin_fast(x)); }
#[test]
fn c1_cubit_cos_taylor_0p7() { let x = cang(); sink(ctrig::cos(x)); }
#[test]
fn c1_cubit_cos_fast_lut_0p7() { let x = cang(); sink(ctrig::cos_fast(x)); }
#[test]
fn c1_cubit_atan_poly_0p7() { let x = cang(); sink(ctrig::atan(x)); }
#[test]
fn c1_cubit_atan_fast_lut_0p7() { let x = cang(); sink(ctrig::atan_fast(x)); }
#[test]
fn c1_cubit_tan_taylor_0p7() { let x = cang(); sink(ctrig::tan(x)); }
#[test]
fn i1_poly_sin5_0p7() { let x = ang(); sink(trig::sin_poly5(x)); }
#[test]
fn i1_poly_sin7_0p7() { let x = ang(); sink(trig::sin_poly7(x)); }
#[test]
fn i1_poly_sin7_2p5() { let x = ang2(); sink(trig::sin_poly7(x)); }
#[test]
fn i1_poly_sin9_0p7() { let x = ang(); sink(trig::sin_poly9(x)); }
#[test]
fn i1_poly_cos7_0p7() { let x = ang(); sink(trig::cos_poly7(x)); }
#[test]
fn i1_bhaskara_sin_0p7() { let x = ang(); sink(trig::sin_bhaskara(x)); }
#[test]
fn i2_poly_atan2_deg11() { let y = ang(); let x = ang2(); sink(trig::atan2_poly11(y, x)); }

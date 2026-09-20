use orion::numbers::fixed_point::implementations::fp8x23wide::core::{
    FP8x23W, FixedTrait, FP8x23WPartialOrd, FP8x23WPartialEq
};

fn max(a: FP8x23W, b: FP8x23W) -> FP8x23W {
    if a >= b {
        a
    } else {
        b
    }
}

fn min(a: FP8x23W, b: FP8x23W) -> FP8x23W {
    if a <= b {
        a
    } else {
        b
    }
}

fn xor(a: FP8x23W, b: FP8x23W) -> bool {
    if (a == FixedTrait::new(0, false) || b == FixedTrait::new(0, false)) && (a != b) {
        true
    } else {
        false
    }
}

fn or(a: FP8x23W, b: FP8x23W) -> bool {
    let zero = FixedTrait::new(0, false);
    if a == zero && b == zero {
        false
    } else {
        true
    }
}

fn and(a: FP8x23W, b: FP8x23W) -> bool {
    let zero = FixedTrait::new(0, false);
    if a == zero || b == zero {
        false
    } else {
        true
    }
}

fn where(a: FP8x23W, b: FP8x23W, c: FP8x23W) -> FP8x23W {
    if a == FixedTrait::new(0, false) {
        c
    } else {
        b
    }
}

fn bitwise_and(a: FP8x23W, b: FP8x23W) -> FP8x23W {
    FixedTrait::new(a.mag & b.mag, a.sign & b.sign)
}

fn bitwise_xor(a: FP8x23W, b: FP8x23W) -> FP8x23W {
    FixedTrait::new(a.mag ^ b.mag, a.sign ^ b.sign)
}

fn bitwise_or(a: FP8x23W, b: FP8x23W) -> FP8x23W {
    FixedTrait::new(a.mag | b.mag, a.sign | b.sign)
}


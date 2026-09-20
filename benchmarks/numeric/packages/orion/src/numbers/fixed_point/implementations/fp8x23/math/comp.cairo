use orion::numbers::fixed_point::implementations::fp8x23::core::{
    FP8x23, FixedTrait, FP8x23PartialOrd, FP8x23PartialEq
};

fn max(a: FP8x23, b: FP8x23) -> FP8x23 {
    if a >= b {
        a
    } else {
        b
    }
}

fn min(a: FP8x23, b: FP8x23) -> FP8x23 {
    if a <= b {
        a
    } else {
        b
    }
}

fn xor(a: FP8x23, b: FP8x23) -> bool {
    if (a == FixedTrait::new(0, false) || b == FixedTrait::new(0, false)) && (a != b) {
        true
    } else {
        false
    }
}

fn or(a: FP8x23, b: FP8x23) -> bool {
    let zero = FixedTrait::new(0, false);

    if a == zero && b == zero {
        false
    } else {
        true
    }
}

fn and(a: FP8x23, b: FP8x23) -> bool {
    let zero = FixedTrait::new(0, false);

    if a == zero || b == zero {
        false
    } else {
        true
    }
}

fn where(a: FP8x23, b: FP8x23, c: FP8x23) -> FP8x23 {
    if a == FixedTrait::new(0, false) {
        c
    } else {
        b
    }
}

fn bitwise_and(a: FP8x23, b: FP8x23) -> FP8x23 {
    FixedTrait::new(a.mag & b.mag, a.sign & b.sign)
}

fn bitwise_xor(a: FP8x23, b: FP8x23) -> FP8x23 {
    FixedTrait::new(a.mag ^ b.mag, a.sign ^ b.sign)
}

fn bitwise_or(a: FP8x23, b: FP8x23) -> FP8x23 {
    FixedTrait::new(a.mag | b.mag, a.sign | b.sign)
}


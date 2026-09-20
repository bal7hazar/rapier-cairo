use orion::numbers::fixed_point::implementations::fp16x16wide::core::{
    FP16x16W, FixedTrait, FP16x16WImpl, FP16x16WPartialOrd, FP16x16WPartialEq
};

fn max(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    if a >= b {
        a
    } else {
        b
    }
}

fn min(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    if a <= b {
        a
    } else {
        b
    }
}

fn xor(a: FP16x16W, b: FP16x16W) -> bool {
    if (a == FixedTrait::new(0, false) || b == FixedTrait::new(0, false)) && (a != b) {
        true
    } else {
        false
    }
}

fn or(a: FP16x16W, b: FP16x16W) -> bool {
    let zero = FixedTrait::new(0, false);
    if a == zero && b == zero {
        false
    } else {
        true
    }
}

fn and(a: FP16x16W, b: FP16x16W) -> bool {
    let zero = FixedTrait::new(0, false);
    if a == zero || b == zero {
        false
    } else {
        true
    }
}

fn where(a: FP16x16W, b: FP16x16W, c: FP16x16W) -> FP16x16W {
    if a == FixedTrait::new(0, false) {
        c
    } else {
        b
    }
}

fn bitwise_and(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    FixedTrait::new(a.mag & b.mag, a.sign & b.sign)
}

fn bitwise_xor(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    FixedTrait::new(a.mag ^ b.mag, a.sign ^ b.sign)
}

fn bitwise_or(a: FP16x16W, b: FP16x16W) -> FP16x16W {
    FixedTrait::new(a.mag | b.mag, a.sign | b.sign)
}


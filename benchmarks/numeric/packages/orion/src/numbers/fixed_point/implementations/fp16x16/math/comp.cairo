use orion::numbers::fixed_point::implementations::fp16x16::core::{
    FP16x16, FixedTrait, FP16x16Impl, FP16x16PartialOrd, FP16x16PartialEq
};

fn max(a: FP16x16, b: FP16x16) -> FP16x16 {
    if a >= b {
        a
    } else {
        b
    }
}

fn min(a: FP16x16, b: FP16x16) -> FP16x16 {
    if a <= b {
        a
    } else {
        b
    }
}

fn xor(a: FP16x16, b: FP16x16) -> bool {
    if (a == FixedTrait::new(0, false) || b == FixedTrait::new(0, false)) && (a != b) {
        true
    } else {
        false
    }
}

fn or(a: FP16x16, b: FP16x16) -> bool {
    let zero = FixedTrait::new(0, false);
    if a == zero && b == zero {
        false
    } else {
        true
    }
}

fn and(a: FP16x16, b: FP16x16) -> bool {
    let zero = FixedTrait::new(0, false);
    if a == zero || b == zero {
        false
    } else {
        true
    }
}

fn where(a: FP16x16, b: FP16x16, c: FP16x16) -> FP16x16 {
    if a == FixedTrait::new(0, false) {
        c
    } else {
        b
    }
}

fn bitwise_and(a: FP16x16, b: FP16x16) -> FP16x16 {
    FixedTrait::new(a.mag & b.mag, a.sign & b.sign)
}

fn bitwise_xor(a: FP16x16, b: FP16x16) -> FP16x16 {
    FixedTrait::new(a.mag ^ b.mag, a.sign ^ b.sign)
}

fn bitwise_or(a: FP16x16, b: FP16x16) -> FP16x16 {
    FixedTrait::new(a.mag | b.mag, a.sign | b.sign)
}


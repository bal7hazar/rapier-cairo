use cubit::f128::types::fixed::{Fixed, FixedTrait, FixedPartialOrd};

fn max(a: Fixed, b: Fixed) -> Fixed {
    if (a >= b) {
        return a;
    } else {
        return b;
    }
}

fn min(a: Fixed, b: Fixed) -> Fixed {
    if (a <= b) {
        return a;
    } else {
        return b;
    }
}


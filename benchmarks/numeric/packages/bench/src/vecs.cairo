//! Generic (monomorphised) small-vector kernels used to compare scalar representations.
use crate::reprs::Fx;

#[derive(Copy, Drop)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}
#[derive(Copy, Drop)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}
#[derive(Copy, Drop)]
pub struct Mat3<T> {
    pub m00: T, pub m01: T, pub m02: T,
    pub m10: T, pub m11: T, pub m12: T,
    pub m20: T, pub m21: T, pub m22: T,
}

pub fn dot2<T, +Add<T>, +Mul<T>, +Copy<T>, +Drop<T>>(a: Vec2<T>, b: Vec2<T>) -> T {
    a.x * b.x + a.y * b.y
}
pub fn cross2<T, +Sub<T>, +Mul<T>, +Copy<T>, +Drop<T>>(a: Vec2<T>, b: Vec2<T>) -> T {
    a.x * b.y - a.y * b.x
}
pub fn length2<T, +Add<T>, +Mul<T>, +Fx<T>, +Copy<T>, +Drop<T>>(a: Vec2<T>) -> T {
    (a.x * a.x + a.y * a.y).sqrt()
}
pub fn normalize2<T, +Add<T>, +Mul<T>, +Div<T>, +Fx<T>, +Copy<T>, +Drop<T>>(a: Vec2<T>) -> Vec2<T> {
    let l = (a.x * a.x + a.y * a.y).sqrt();
    Vec2 { x: a.x / l, y: a.y / l }
}
pub fn mat3_mul_vec3<T, +Add<T>, +Mul<T>, +Copy<T>, +Drop<T>>(m: Mat3<T>, v: Vec3<T>) -> Vec3<T> {
    Vec3 {
        x: m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
        y: m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
        z: m.m20 * v.x + m.m21 * v.y + m.m22 * v.z,
    }
}

// ---- fused (single rescale per output) kernels -------------------------------------------
pub mod fused_i64b {
    use crate::reprs::Fx;
    use crate::reprs::i64b::{I64b, fused2, fused3, fused_diff};
    use super::{Mat3, Vec2, Vec3};

    pub fn dot2(a: Vec2<I64b>, b: Vec2<I64b>) -> I64b {
        fused2(a.x, b.x, a.y, b.y)
    }
    pub fn cross2(a: Vec2<I64b>, b: Vec2<I64b>) -> I64b {
        fused_diff(a.x, b.y, a.y, b.x)
    }
    pub fn length2(a: Vec2<I64b>) -> I64b {
        fused2(a.x, a.x, a.y, a.y).sqrt()
    }
    /// sqrt taken directly on the 2^64-scaled sum of squares (no rescale, no squared-length overflow)
    pub fn length2_wide(a: Vec2<I64b>) -> I64b {
        crate::reprs::i64b::length_wide2(a.x, a.y)
    }
    pub fn normalize2_wide(a: Vec2<I64b>) -> Vec2<I64b> {
        let l = crate::reprs::i64b::length_wide2(a.x, a.y);
        Vec2 { x: a.x / l, y: a.y / l }
    }
    /// fused length + 2 divisions
    pub fn normalize2(a: Vec2<I64b>) -> Vec2<I64b> {
        let l = fused2(a.x, a.x, a.y, a.y).sqrt();
        Vec2 { x: a.x / l, y: a.y / l }
    }
    /// fused squared length, inverse sqrt (1 u128 div + core sqrt), 2 multiplications.
    /// Precision of 1/len is 32 fractional bits *absolute*: relative error grows with len.
    pub fn normalize2_rsqrt(a: Vec2<I64b>) -> Vec2<I64b> {
        let l2 = fused2(a.x, a.x, a.y, a.y);
        let m: u64 = l2.v.try_into().unwrap();
        let inv = I64b { v: crate::sqrts::inv_sqrt_div_first(m).try_into().unwrap() };
        Vec2 { x: a.x * inv, y: a.y * inv }
    }
    pub fn mat3_mul_vec3(m: Mat3<I64b>, v: Vec3<I64b>) -> Vec3<I64b> {
        Vec3 {
            x: fused3(m.m00, v.x, m.m01, v.y, m.m02, v.z),
            y: fused3(m.m10, v.x, m.m11, v.y, m.m12, v.z),
            z: fused3(m.m20, v.x, m.m21, v.y, m.m22, v.z),
        }
    }
}

pub mod fused_fe {
    use crate::reprs::Fx;
    use crate::reprs::felt::{Fe, rescale};
    use super::{Mat3, Vec2, Vec3};

    pub fn dot2(a: Vec2<Fe>, b: Vec2<Fe>) -> Fe {
        rescale(a.x.v * b.x.v + a.y.v * b.y.v)
    }
    pub fn cross2(a: Vec2<Fe>, b: Vec2<Fe>) -> Fe {
        rescale(a.x.v * b.y.v - a.y.v * b.x.v)
    }
    pub fn length2(a: Vec2<Fe>) -> Fe {
        rescale(a.x.v * a.x.v + a.y.v * a.y.v).sqrt()
    }
    pub fn mat3_mul_vec3(m: Mat3<Fe>, v: Vec3<Fe>) -> Vec3<Fe> {
        Vec3 {
            x: rescale(m.m00.v * v.x.v + m.m01.v * v.y.v + m.m02.v * v.z.v),
            y: rescale(m.m10.v * v.x.v + m.m11.v * v.y.v + m.m12.v * v.z.v),
            z: rescale(m.m20.v * v.x.v + m.m21.v * v.y.v + m.m22.v * v.z.v),
        }
    }
}

// ---- Mat3 x Mat3 (section E) ---------------------------------------------------------------
pub fn mat3_mul_mat3<T, +Add<T>, +Mul<T>, +Copy<T>, +Drop<T>>(a: Mat3<T>, b: Mat3<T>) -> Mat3<T> {
    Mat3 {
        m00: a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20,
        m01: a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21,
        m02: a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22,
        m10: a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20,
        m11: a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21,
        m12: a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22,
        m20: a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20,
        m21: a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21,
        m22: a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22,
    }
}
pub fn mat3_mul_mat3_fused_i64b(
    a: Mat3<crate::reprs::i64b::I64b>, b: Mat3<crate::reprs::i64b::I64b>,
) -> Mat3<crate::reprs::i64b::I64b> {
    Mat3 {
        m00: crate::reprs::i64b::fused3(a.m00, b.m00, a.m01, b.m10, a.m02, b.m20),
        m01: crate::reprs::i64b::fused3(a.m00, b.m01, a.m01, b.m11, a.m02, b.m21),
        m02: crate::reprs::i64b::fused3(a.m00, b.m02, a.m01, b.m12, a.m02, b.m22),
        m10: crate::reprs::i64b::fused3(a.m10, b.m00, a.m11, b.m10, a.m12, b.m20),
        m11: crate::reprs::i64b::fused3(a.m10, b.m01, a.m11, b.m11, a.m12, b.m21),
        m12: crate::reprs::i64b::fused3(a.m10, b.m02, a.m11, b.m12, a.m12, b.m22),
        m20: crate::reprs::i64b::fused3(a.m20, b.m00, a.m21, b.m10, a.m22, b.m20),
        m21: crate::reprs::i64b::fused3(a.m20, b.m01, a.m21, b.m11, a.m22, b.m21),
        m22: crate::reprs::i64b::fused3(a.m20, b.m02, a.m21, b.m12, a.m22, b.m22),
    }
}

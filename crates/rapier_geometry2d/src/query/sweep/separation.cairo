//! The 2D separation functions of the swept time of impact (Parry `sweep_toi/separation.rs`, from
//! Box2D's `b2SeparationFunction`): the signed distance of two swept proxies along an axis fixed
//! by the features of their closest pair at `t1` (a vertex pair, or a face of one against a
//! vertex of the other), as a function of the sweep fraction.

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2Trait;
use rapier_math::rot2::Rot2Trait;
use crate::point::wide2::dot_wide;
use super::proxy::SimplexCache;
use super::super::normalize_and_length;
use super::{Sweep, SweepTrait, ToiProxy, ToiProxyTrait};

/// Upstream's `INVALID_INDEX`: the face side of a face separation has no support index.
pub const INVALID_INDEX: u32 = 0xffffffff;

/// Which features fix the axis.
#[derive(Copy, Drop, Debug, PartialEq)]
pub enum SeparationType {
    Points,
    FaceA,
    FaceB,
}

/// A separation function (upstream `SeparationFunction`, 2D).
#[derive(Copy, Drop, Debug)]
pub struct SeparationFunction {
    pub proxy_a: ToiProxy,
    pub proxy_b: ToiProxy,
    pub sweep_a: Sweep,
    pub sweep_b: Sweep,
    /// The face's midpoint in its proxy's frame (`Face*` only).
    pub local_point: Vec2,
    /// World axis (`Points`) or the face's unit normal in its proxy's frame (`Face*`).
    pub axis: Vec2,
    pub ty: SeparationType,
}

#[inline(always)]
fn dot(a: Vec2, b: Vec2) -> Fixed {
    fixed::wide::dot2(a.x, b.x, a.y, b.y)
}

/// `normalize_or_zero`.
#[inline(always)]
fn unit(v: Vec2) -> Vec2 {
    let (n, _) = normalize_and_length(v);
    n.unwrap_or(Vec2 { x: ZERO, y: ZERO })
}

#[inline(always)]
fn point(proxy: ToiProxy, index: u32) -> Vec2 {
    *proxy.points[index]
}

#[generate_trait]
pub impl SeparationFunctionImpl of SeparationFunctionTrait {
    /// The separation function of the closest features `cache` at `t1` (upstream `new`):
    /// `count = 1` a vertex pair (world axis between them), otherwise the face of the side whose
    /// two indices differ, its normal turned towards the other side's vertex.
    fn new(
        cache: SimplexCache,
        proxy_a: ToiProxy,
        sweep_a: Sweep,
        proxy_b: ToiProxy,
        sweep_b: Sweep,
        t1: Fixed,
    ) -> SeparationFunction {
        let xf_a = sweep_a.transform_at(t1);
        let xf_b = sweep_b.transform_at(t1);
        let [ia0, ia1] = cache.index_a;
        let [ib0, ib1] = cache.index_b;
        if cache.count == 1 {
            let point_a = xf_a.transform_point(point(proxy_a, ia0));
            let point_b = xf_b.transform_point(point(proxy_b, ib0));
            return SeparationFunction {
                proxy_a,
                proxy_b,
                sweep_a,
                sweep_b,
                local_point: Vec2 { x: ZERO, y: ZERO },
                axis: unit(point_b - point_a),
                ty: SeparationType::Points,
            };
        }
        if ia0 == ia1 {
            let (b1, b2) = (point(proxy_b, ib0), point(proxy_b, ib1));
            let edge = b2 - b1;
            let mut axis = unit(Vec2 { x: edge.y, y: -edge.x });
            let normal = xf_b.rotation.rotate(axis);
            let local_point = (b1 + b2).mul_scalar(fixed::HALF);
            let point_b = xf_b.transform_point(local_point);
            let point_a = xf_a.transform_point(point(proxy_a, ia0));
            let d = point_a - point_b;
            if dot_wide(d.x, d.y, normal.x, normal.y) < 0 {
                axis = -axis;
            }
            SeparationFunction {
                proxy_a, proxy_b, sweep_a, sweep_b, local_point, axis, ty: SeparationType::FaceB,
            }
        } else {
            let (a1, a2) = (point(proxy_a, ia0), point(proxy_a, ia1));
            let edge = a2 - a1;
            let mut axis = unit(Vec2 { x: edge.y, y: -edge.x });
            let normal = xf_a.rotation.rotate(axis);
            let local_point = (a1 + a2).mul_scalar(fixed::HALF);
            let point_a = xf_a.transform_point(local_point);
            let point_b = xf_b.transform_point(point(proxy_b, ib0));
            let d = point_b - point_a;
            if dot_wide(d.x, d.y, normal.x, normal.y) < 0 {
                axis = -axis;
            }
            SeparationFunction {
                proxy_a, proxy_b, sweep_a, sweep_b, local_point, axis, ty: SeparationType::FaceA,
            }
        }
    }

    /// The deepest points along the axis at `t` and their separation (upstream
    /// `find_min_separation`): `(separation, index_a, index_b)`, `INVALID_INDEX` for a face side.
    fn find_min_separation(self: @SeparationFunction, t: Fixed) -> (Fixed, u32, u32) {
        let (proxy_a, proxy_b) = (*self.proxy_a, *self.proxy_b);
        let xf_a = (*self.sweep_a).transform_at(t);
        let xf_b = (*self.sweep_b).transform_at(t);
        let axis = *self.axis;
        match *self.ty {
            SeparationType::Points => {
                let index_a = proxy_a.support(xf_a.rotation.inverse_rotate(axis));
                let index_b = proxy_b.support(xf_b.rotation.inverse_rotate(-axis));
                let point_a = xf_a.transform_point(point(proxy_a, index_a));
                let point_b = xf_b.transform_point(point(proxy_b, index_b));
                (dot(point_b - point_a, axis), index_a, index_b)
            },
            SeparationType::FaceA => {
                let normal = xf_a.rotation.rotate(axis);
                let point_a = xf_a.transform_point(*self.local_point);
                let index_b = proxy_b.support(xf_b.rotation.inverse_rotate(-normal));
                let point_b = xf_b.transform_point(point(proxy_b, index_b));
                (dot(point_b - point_a, normal), INVALID_INDEX, index_b)
            },
            SeparationType::FaceB => {
                let normal = xf_b.rotation.rotate(axis);
                let point_b = xf_b.transform_point(*self.local_point);
                let index_a = proxy_a.support(xf_a.rotation.inverse_rotate(-normal));
                let point_a = xf_a.transform_point(point(proxy_a, index_a));
                (dot(point_a - point_b, normal), index_a, INVALID_INDEX)
            },
        }
    }

    /// The separation of the given support points at `t` (upstream `evaluate`).
    fn evaluate(self: @SeparationFunction, index_a: u32, index_b: u32, t: Fixed) -> Fixed {
        let (proxy_a, proxy_b) = (*self.proxy_a, *self.proxy_b);
        let xf_a = (*self.sweep_a).transform_at(t);
        let xf_b = (*self.sweep_b).transform_at(t);
        let axis = *self.axis;
        match *self.ty {
            SeparationType::Points => {
                let point_a = xf_a.transform_point(point(proxy_a, index_a));
                let point_b = xf_b.transform_point(point(proxy_b, index_b));
                dot(point_b - point_a, axis)
            },
            SeparationType::FaceA => {
                let normal = xf_a.rotation.rotate(axis);
                let point_a = xf_a.transform_point(*self.local_point);
                let point_b = xf_b.transform_point(point(proxy_b, index_b));
                dot(point_b - point_a, normal)
            },
            SeparationType::FaceB => {
                let normal = xf_b.rotation.rotate(axis);
                let point_b = xf_b.transform_point(*self.local_point);
                let point_a = xf_a.transform_point(point(proxy_a, index_a));
                dot(point_a - point_b, normal)
            },
        }
    }
}

//! Sweep-local body store (BT1): velocities in the dictionary (3 felts per entry), poses in an
//! array rebuilt once per substep by `integrate`, masses and handles frozen in the initial span.
//! Contact sweeps only read poses and write velocities. The `DenseBodiesTrait` view serves the
//! joint sweeps: `get` assembles the whole body, `set_pair` stores velocities only (the joint
//! sweeps change nothing else). `WORLD` reads as the identity body and ignores writes.
use core::dict::{Felt252Dict, Felt252DictTrait};
use core::nullable::{FromNullableResult, NullableTrait, match_nullable};
use fixed::{Fixed, MAX};
use glam::Vec2Trait;
use rapier_math::pose2::Pose2;
use crate::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use super::super::super::super::body::{SolverBody, SolverVel, WORLD};
use super::super::super::super::body_store::{BodyStep, DenseBodiesTrait, errors};

#[derive(Destruct)]
pub(crate) struct SweepBodies {
    vels: Felt252Dict<Nullable<SolverVel>>,
    pub(crate) poses: Array<Pose2>,
    frames: Span<SolverBody>,
}

#[generate_trait]
pub(crate) impl SweepBodiesImpl of SweepBodiesTrait {
    /// Velocity of dense body `i`; zero for `WORLD`.
    #[inline(always)]
    fn vel(ref self: SweepBodies, i: u32) -> SolverVel {
        if i == WORLD {
            return Default::default();
        }
        match match_nullable(self.vels.get(i.into())) {
            FromNullableResult::NotNull(v) => v.unbox(),
            FromNullableResult::Null => core::panic_with_felt252(errors::INDEX),
        }
    }
    /// Store both velocities; `WORLD` writes are ignored, the second wins when ids coincide.
    #[inline(always)]
    fn set_vels(ref self: SweepBodies, i: u32, v1: SolverVel, j: u32, v2: SolverVel) {
        if i != WORLD {
            self.vels.insert(i.into(), NullableTrait::new(v1));
        }
        if j != WORLD {
            self.vels.insert(j.into(), NullableTrait::new(v2));
        }
    }
    /// `island::add_forces`: the substep force increment of every moving body.
    fn add_forces(ref self: SweepBodies, steps: Span<BodyStep>) {
        let n = self.frames.len();
        let mut i = 0;
        while i != n {
            let step = steps.at(i);
            if *step.moving {
                let mut v = self.vel(i);
                let dv = *step.increment;
                v.linear = v.linear + dv.linvel;
                v.angular += dv.angvel;
                self.vels.insert(i.into(), NullableTrait::new(v));
            }
            i += 1;
        }
    }
    /// `island::integrate`: velocity caps then the pose update of every moving body.
    fn integrate(
        ref self: SweepBodies, steps: Span<BodyStep>, dt: Fixed, max_lin: Fixed, max_ang: Fixed,
    ) {
        let mut out = array![];
        let mut i = 0;
        while let Some(pose) = self.poses.pop_front() {
            if *steps.at(i).moving {
                let mut v = self.vel(i);
                // Sentinel guard is before length computation: disabled caps need no sqrt.
                if max_lin != MAX {
                    let length = v.linear.length();
                    if length > max_lin {
                        v.linear = v.linear.mul_scalar(max_lin / length);
                    }
                }
                if v.angular > max_ang {
                    v.angular = max_ang;
                }
                if v.angular < -max_ang {
                    v.angular = -max_ang;
                }
                let rv = RigidBodyVelocity { linvel: v.linear, angvel: v.angular };
                out.append(rv.integrate(dt, pose, Default::default()));
                self.vels.insert(i.into(), NullableTrait::new(v));
            } else {
                out.append(pose);
            }
            i += 1;
        }
        self.poses = out;
    }
    /// `island::damp`: full-step damping of every moving body's velocities.
    fn damp(ref self: SweepBodies, steps: Span<BodyStep>, dt: Fixed) {
        let n = self.frames.len();
        let mut i = 0;
        while i != n {
            let step = steps.at(i);
            if *step.moving {
                let v = self.vel(i);
                let d = RigidBodyVelocity { linvel: v.linear, angvel: v.angular }
                    .apply_damping(dt, *step.damping);
                self
                    .vels
                    .insert(
                        i.into(),
                        NullableTrait::new(SolverVel { linear: d.linvel, angular: d.angvel }),
                    );
            }
            i += 1;
        }
    }
    /// Write every body's velocities and pose back into `bodies`.
    fn finish<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref self: SweepBodies, ref bodies: B) {
        let mut frames = self.frames;
        let mut poses = self.poses.span();
        let mut i = 0;
        while let Some(frame) = frames.pop_front() {
            let v = self.vel(i);
            let b = SolverBody {
                position: *poses.pop_front().unwrap(),
                linvel: v.linear,
                angvel: v.angular,
                ..*frame,
            };
            bodies.set_pair(i, b, WORLD, Default::default());
            i += 1;
        }
    }
}

pub(crate) impl SweepBodiesDense of DenseBodiesTrait<SweepBodies> {
    fn new(bodies: Span<SolverBody>) -> SweepBodies {
        let mut vels: Felt252Dict<Nullable<SolverVel>> = Default::default();
        let mut poses = array![];
        let mut i: u32 = 0;
        for b in bodies {
            vels
                .insert(
                    i.into(),
                    NullableTrait::new(SolverVel { linear: *b.linvel, angular: *b.angvel }),
                );
            poses.append(*b.position);
            i += 1;
        }
        SweepBodies { vels, poses, frames: bodies }
    }
    fn len(self: @SweepBodies) -> u32 {
        self.frames.len()
    }
    fn get(ref self: SweepBodies, index: u32) -> SolverBody {
        if index == WORLD {
            return Default::default();
        }
        assert(index < self.frames.len(), errors::INDEX);
        let v = self.vel(index);
        SolverBody {
            position: *self.poses.at(index),
            linvel: v.linear,
            angvel: v.angular,
            ..*self.frames.at(index),
        }
    }
    fn set_pair(ref self: SweepBodies, i: u32, a: SolverBody, j: u32, b: SolverBody) {
        let n = self.frames.len();
        assert((i == WORLD || i < n) && (j == WORLD || j < n), errors::INDEX);
        self
            .set_vels(
                i,
                SolverVel { linear: a.linvel, angular: a.angvel },
                j,
                SolverVel { linear: b.linvel, angular: b.angvel },
            );
    }
}

//! Sequential whole-world soft-step driver (D8/D9). Manifolds and joints retain caller order;
//! each sweep solves joints before contacts. No sleeping, island discovery, CCD or colouring.
//! Contact sweeps use cached frame coefficients and two body values; joint sweeps retain
//! their measured array adapter. Global scattering remains O(1).
//! With contacts (BT1, `sweeps::split`), constraints are generated once into a frozen span and a
//! small hot array, and the step runs on a `SweepBodies` store (velocities in the dictionary,
//! poses in a per-substep array), written back into `bodies` at the end: same arithmetic,
//! results bit for bit, about half the Cairo steps of the constraint-set sweeps.
mod empty;
mod sweeps;
use fixed::{Fixed, MAX, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::RigidBodyDamping;
use rapier_geometry2d::contact::ContactManifold;
use sweeps::array_joint::joints;
#[cfg(test)]
use sweeps::contact::contacts;
use sweeps::split::{BankPoint, Frozen, FrozenPoint, HotPoint, SweepBodies, SweepBodiesTrait};
use sweeps::{prepare_joints, rebuild_joints, split};
use crate::joint::ImpulseJoint;
use crate::rigid_body::{RigidBodyVelocity, RigidBodyVelocityTrait};
use crate::rigid_body_set::RigidBody;
use super::body::{SolverBody, WORLD};
use super::body_store::{
    BodyStep, DenseBodies, DenseBodiesTrait, SolverBodyStore, gather, writeback,
};
#[cfg(test)]
use super::contact::ContactConstraintsSetTrait;

/// Invalid timestep/velocity cap. Parameter and fixed-point panics otherwise propagate.
pub mod errors {
    /// The timestep or a scaled velocity limit is negative.
    pub const NEGATIVE: felt252 = 'Island: negative parameter';
}

/// Advance one step on a store built with these parameters, persisting impulses into the
/// ordered manifold/joint arrays. `contact_set` contains frozen manifold membership for this
/// frame (flatten DD pairs in pair-slot order); `joint_set` contains body-local joint frames.
/// Applies force increments, joint rebuilding, contact warmstarts, biased PGS, velocity caps,
/// linearized integration, relaxed PGS, restitution, impulse writeback, then full-step damping.
/// Call `store.to_bodies` afterwards; do not reuse this per-step scratch for another step.
/// Products floor, divisions round to nearest, rotations renormalize. All intermediates must fit
/// Q32.32. Zero dt is a no-op; zero solver iterations and negative dt/caps panic.
pub fn solve_island(
    params: IntegrationParameters,
    ref store: SolverBodyStore,
    ref contact_set: Array<ContactManifold>,
    ref joint_set: Array<ImpulseJoint>,
) {
    run(params, ref store.bodies, store.steps, ref contact_set, ref joint_set);
}

/// The input of [`solve_island_input`]: one solver body and its frozen step data per entry, in
/// entry order (BT4: gathered straight from the entries, no `SolverBodyStore`).
#[derive(Drop)]
pub struct SolverInput {
    bodies: Array<SolverBody>,
    steps: Array<BodyStep>,
}

/// The impulses a solve left in one contact point (the fields `solve_island` writes back into
/// the manifold's point `contact_id`).
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub struct PointImpulses {
    pub contact_id: u8,
    pub impulse: Fixed,
    pub tangent_impulse: Fixed,
    pub warmstart_impulse: Fixed,
    pub warmstart_tangent_impulse: Fixed,
}

/// The impulses of one manifold: `count` points (0 when no constraint was generated for it:
/// then `solve_island` leaves the manifold unchanged).
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub struct ManifoldImpulses {
    pub count: u8,
    pub a: PointImpulses,
    pub b: PointImpulses,
}

/// What [`solve_island_input`] leaves: the solved bodies in input order and one
/// [`ManifoldImpulses`] per input manifold (none when there was no contact constraint).
#[derive(Drop)]
pub struct SolvedIsland {
    bodies: Span<SolverBody>,
    steps: Span<BodyStep>,
    impulses: Span<ManifoldImpulses>,
}

/// The result of a solve that did not run (no constraint): no body, no impulse.
pub impl SolvedIslandDefault of Default<SolvedIsland> {
    fn default() -> SolvedIsland {
        SolvedIsland { bodies: array![].span(), steps: array![].span(), impulses: array![].span() }
    }
}

#[generate_trait]
pub impl SolverInputImpl of SolverInputTrait {
    /// `SolverBodyStoreTrait::from_entries` without the store: the same gather, same panics.
    fn gather(
        entries: Span<(Handle, RigidBody)>, gravity: Vec2, params: IntegrationParameters,
    ) -> SolverInput {
        let dt = params.substep_dt();
        let mut input = Self::new();
        for (handle, rb) in entries {
            input.push(*handle, *rb, gravity, dt);
        }
        input
    }

    /// An empty input, filled body by body with [`push`](Self::push).
    #[inline(always)]
    fn new() -> SolverInput {
        SolverInput { bodies: array![], steps: array![] }
    }

    /// Appends the body `rb` of `handle` as `gather` does, `dt` being `params.substep_dt()`.
    #[inline(always)]
    fn push(ref self: SolverInput, handle: Handle, rb: RigidBody, gravity: Vec2, dt: Fixed) {
        let (body, step) = gather(handle, rb, gravity, dt);
        self.bodies.append(body);
        self.steps.append(step);
    }
}

#[generate_trait]
pub impl SolvedIslandImpl of SolvedIslandTrait {
    /// `SolverBodyStoreTrait::write_body` of the input body `index` (a non-moving body is left
    /// unchanged). Products floor.
    fn write_body(self: @SolvedIsland, index: u32, ref rb: RigidBody) {
        let step = *self.steps.at(index);
        if step.moving {
            writeback(*self.bodies.at(index), step, ref rb);
        }
    }

    /// Writes the impulses of the input manifold `index` into `manifold` (its copy in the pair
    /// list), as `solve_island` writes them into its manifold array. Nothing without impulses.
    fn write_impulses(self: @SolvedIsland, index: u32, ref manifold: ContactManifold) {
        if let Some(impulses) = self.impulses.get(index) {
            let impulses = *impulses.unbox();
            if impulses.count != 0 {
                write_point(impulses.a, ref manifold);
                if impulses.count == 2 {
                    write_point(impulses.b, ref manifold);
                }
            }
        }
    }
}

/// `split::writeback`'s write of one point.
#[inline(always)]
fn write_point(p: PointImpulses, ref m: ContactManifold) {
    let [mut p0, mut p1] = m.points;
    if p.contact_id == 0 {
        p0.data.impulse = p.impulse;
        p0.data.tangent_impulse = p.tangent_impulse;
        p0.data.warmstart_impulse = p.warmstart_impulse;
        p0.data.warmstart_tangent_impulse = p.warmstart_tangent_impulse;
    } else {
        p1.data.impulse = p.impulse;
        p1.data.tangent_impulse = p.tangent_impulse;
        p1.data.warmstart_impulse = p.warmstart_impulse;
        p1.data.warmstart_tangent_impulse = p.warmstart_tangent_impulse;
    }
    m.points = [p0, p1];
}

/// The impulses of one point from the sweeps' state (`split::write_point`'s values).
#[inline(always)]
fn point_impulses(h: HotPoint, b: BankPoint, f: @FrozenPoint) -> PointImpulses {
    PointImpulses {
        contact_id: *f.contact_id,
        impulse: b.acc + h.impulse,
        tangent_impulse: b.t_acc + h.t_impulse,
        warmstart_impulse: h.impulse,
        warmstart_tangent_impulse: h.t_impulse,
    }
}

/// Collects the bodies `SweepBodiesTrait::finish` writes, in dense order (only `set_pair(i, b,
/// WORLD, _)` with `i` ascending from 0 is used).
#[derive(Drop)]
struct Collected {
    bodies: Array<SolverBody>,
}

impl CollectedDense of DenseBodiesTrait<Collected> {
    fn new(bodies: Span<SolverBody>) -> Collected {
        Collected { bodies: array![] }
    }

    fn len(self: @Collected) -> u32 {
        self.bodies.len()
    }
    fn get(ref self: Collected, index: u32) -> SolverBody {
        *self.bodies.at(index)
    }
    fn set_pair(ref self: Collected, i: u32, a: SolverBody, j: u32, b: SolverBody) {
        self.bodies.append(a);
    }
}

/// [`solve_island`] on gathered entries (BT4): the same stages, arithmetic and panics, with the
/// bodies taken from `input` instead of a `SolverBodyStore` and the contact impulses returned per
/// manifold instead of written into a manifold array (the pipeline writes them into its pairs).
/// `manifolds` are the frozen contact set in solve order; `joint_set` as in `solve_island`.
pub fn solve_island_input(
    params: IntegrationParameters,
    input: SolverInput,
    manifolds: Span<ContactManifold>,
    ref joint_set: Array<ImpulseJoint>,
) -> SolvedIsland {
    let SolverInput { bodies: initial, steps } = input;
    let steps = steps.span();
    let dt = params.substep_dt();
    let max_lin = params.max_linear_velocity();
    let max_corrective = params.max_corrective_velocity();
    assert(params.dt >= ZERO && max_lin >= ZERO && max_corrective >= ZERO, errors::NEGATIVE);
    if params.dt == ZERO {
        return SolvedIsland { bodies: initial.span(), steps, impulses: array![].span() };
    }
    let max_ang = Fixed { raw: 3373259426 } * params.inv_dt();
    let initial = initial.span();
    let (frozen, mut state) = if manifolds.is_empty() {
        (array![], split::State { hot: array![], bank: array![], bounce: false })
    } else {
        split::generation::generate(manifolds, initial, params, dt)
    };
    let builders = prepare_joints(joint_set.span(), initial, steps);
    if frozen.is_empty() {
        // No contact constraint: the joint-only stages on the dense store, as `solve_island`.
        let mut bodies: DenseBodies = DenseBodiesTrait::new(initial);
        empty::run(params, ref bodies, steps, builders.span(), ref joint_set, dt, max_lin, max_ang);
        return SolvedIsland {
            bodies: snapshot(ref bodies).span(), steps, impulses: array![].span(),
        };
    }
    let frozen = frozen.span();
    let mut sb: SweepBodies = DenseBodiesTrait::new(initial);
    let mut rows = array![];
    let mut substep = 0;
    while substep != params.num_solver_iterations {
        sb.add_forces(steps);
        rows = rebuild_joints(ref sb, builders.span(), rows.span(), params, substep != 0);
        let update = if substep != 0 && params.num_internal_stabilization_iterations != 0 {
            5
        } else {
            0
        };
        split::contacts(ref state, frozen, ref sb, params, update);
        let mut i = 0;
        while i != params.num_internal_pgs_iterations {
            joints(ref rows, ref sb, true, params.warmstart_joints && i == 0);
            split::contacts(ref state, frozen, ref sb, params, 1);
            i += 1;
        }
        sb.integrate(steps, dt, max_lin, max_ang);
        let mut i = 0;
        while i != params.num_internal_stabilization_iterations {
            joints(ref rows, ref sb, false, false);
            split::contacts(ref state, frozen, ref sb, params, if i == 0 {
                2
            } else {
                3
            });
            i += 1;
        }
        substep += 1;
    }
    split::contacts(ref state, frozen, ref sb, params, 4);
    let impulses = impulses_of(frozen, @state, manifolds.len());
    sweeps::write_joints(rows.span(), ref joint_set);
    sb.damp(steps, params.dt);
    let mut out = Collected { bodies: array![] };
    sb.finish(ref out);
    SolvedIsland { bodies: out.bodies.span(), steps, impulses }
}

/// One [`ManifoldImpulses`] per manifold id below `n` from the split state (`split::writeback`'s
/// values, the active constraints in ascending manifold id).
fn impulses_of(mut frozen: Span<Frozen>, state: @split::State, n: u32) -> Span<ManifoldImpulses> {
    let mut hot = state.hot.span();
    let mut bank = state.bank.span();
    let mut out = array![];
    let mut id = 0;
    while id != n {
        let mut record: ManifoldImpulses = Default::default();
        if let Some(f) = frozen.get(0) {
            let f = f.unbox();
            if *f.manifold_id == id {
                let _ = frozen.pop_front();
                let h = *hot.pop_front().unwrap();
                let b = *bank.pop_front().unwrap();
                record.count = *f.count;
                record.a = point_impulses(h.a, b.a, f.a);
                if *f.count == 2 {
                    record.b = point_impulses(h.b, b.b, f.b);
                }
            }
        }
        out.append(record);
        id += 1;
    }
    out.span()
}

fn snapshot<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B) -> Array<SolverBody> {
    let n = bodies.len();
    let mut i = 0;
    let mut out = array![];
    while i != n {
        out.append(bodies.get(i));
        i += 1;
    }
    out
}

fn run<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    params: IntegrationParameters,
    ref bodies: B,
    steps: Span<BodyStep>,
    ref manifolds: Array<ContactManifold>,
    ref joint_set: Array<ImpulseJoint>,
) {
    let dt = params.substep_dt();
    let max_lin = params.max_linear_velocity();
    let max_corrective = params.max_corrective_velocity();
    assert(params.dt >= ZERO && max_lin >= ZERO && max_corrective >= ZERO, errors::NEGATIVE);
    if params.dt == ZERO {
        return;
    }
    // π/4 per FULL frame, exactly upstream's MAX_ROTATION policy.
    let max_ang = Fixed { raw: 3373259426 } * params.inv_dt();
    let initial = snapshot(ref bodies);
    if manifolds.is_empty() {
        let builders = prepare_joints(joint_set.span(), initial.span(), steps);
        empty::run(params, ref bodies, steps, builders.span(), ref joint_set, dt, max_lin, max_ang);
        return;
    }
    let (frozen, mut state) = split::generation::generate(
        manifolds.span(), initial.span(), params, dt,
    );
    let builders = prepare_joints(joint_set.span(), initial.span(), steps);
    if frozen.is_empty() {
        empty::run(params, ref bodies, steps, builders.span(), ref joint_set, dt, max_lin, max_ang);
        return;
    }
    let frozen = frozen.span();
    let mut sb: SweepBodies = DenseBodiesTrait::new(initial.span());
    let mut rows = array![];
    let mut substep = 0;
    while substep != params.num_solver_iterations {
        sb.add_forces(steps);
        rows = rebuild_joints(ref sb, builders.span(), rows.span(), params, substep != 0);
        // BT3: after the first substep, a refresh (stage 2) ran on the current poses.
        let update = if substep != 0 && params.num_internal_stabilization_iterations != 0 {
            5
        } else {
            0
        };
        split::contacts(ref state, frozen, ref sb, params, update);
        let mut i = 0;
        while i != params.num_internal_pgs_iterations {
            joints(ref rows, ref sb, true, params.warmstart_joints && i == 0);
            split::contacts(ref state, frozen, ref sb, params, 1);
            i += 1;
        }
        sb.integrate(steps, dt, max_lin, max_ang);
        let mut i = 0;
        while i != params.num_internal_stabilization_iterations {
            joints(ref rows, ref sb, false, false);
            split::contacts(ref state, frozen, ref sb, params, if i == 0 {
                2
            } else {
                3
            });
            i += 1;
        }
        substep += 1;
    }
    split::contacts(ref state, frozen, ref sb, params, 4);
    split::writeback(frozen, @state, ref manifolds);
    sweeps::write_joints(rows.span(), ref joint_set);
    sb.damp(steps, params.dt);
    sb.finish(ref bodies);
}

fn add_forces<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B, steps: Span<BodyStep>) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            add_force(ref b, *steps.at(i).increment);
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
fn integrate<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref bodies: B, steps: Span<BodyStep>, dt: Fixed, max_lin: Fixed, max_ang: Fixed,
) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            integrate_body(ref b, dt, max_lin, max_ang);
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
fn damp<B, +DenseBodiesTrait<B>, +Destruct<B>>(ref bodies: B, steps: Span<BodyStep>, dt: Fixed) {
    let mut i = 0;
    let n = bodies.len();
    while i != n {
        if *steps.at(i).moving {
            let mut b = bodies.get(i);
            damp_body(ref b, *steps.at(i).damping, dt);
            bodies.set_pair(i, b, WORLD, Default::default());
        }
        i += 1;
    }
}
/// One body's share of `add_forces`.
#[inline(always)]
fn add_force(ref b: SolverBody, dv: RigidBodyVelocity) {
    b.linvel = b.linvel + dv.linvel;
    b.angvel += dv.angvel;
}
/// One body's share of `integrate`: velocity caps, then the pose update.
#[inline(always)]
fn integrate_body(ref b: SolverBody, dt: Fixed, max_lin: Fixed, max_ang: Fixed) {
    // Sentinel guard is before length computation: disabled caps need no sqrt.
    if max_lin != MAX {
        let length = b.linvel.length();
        if length > max_lin {
            b.linvel = b.linvel.mul_scalar(max_lin / length);
        }
    }
    if b.angvel > max_ang {
        b.angvel = max_ang;
    }
    if b.angvel < -max_ang {
        b.angvel = -max_ang;
    }
    let v = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel };
    b.position = v.integrate(dt, b.position, Default::default());
}
/// One body's share of `damp`.
#[inline(always)]
fn damp_body(ref b: SolverBody, damping: RigidBodyDamping, dt: Fixed) {
    let v = RigidBodyVelocity { linvel: b.linvel, angvel: b.angvel }.apply_damping(dt, damping);
    b.linvel = v.linvel;
    b.angvel = v.angvel;
}

/// Per-step constants of [`FreeBodySolverTrait::solve`], computed once per step. `Default` is
/// a placeholder for a step without free body: zero dt, it integrates nothing.
#[derive(Copy, Drop, Debug, PartialEq, Default)]
pub struct FreeBodySolver {
    gravity: Vec2,
    full_dt: Fixed,
    dt: Fixed,
    max_lin: Fixed,
    max_ang: Fixed,
    iterations: u32,
}

#[generate_trait]
pub impl FreeBodySolverImpl of FreeBodySolverTrait {
    /// The constants `solve_island` derives from `params`. Panics as `solve_island` does on
    /// zero solver iterations (`IntegrationParameters`) and negative parameters
    /// (`Island: negative parameter`).
    fn new(params: IntegrationParameters, gravity: Vec2) -> FreeBodySolver {
        let dt = params.substep_dt();
        let max_lin = params.max_linear_velocity();
        let max_corrective = params.max_corrective_velocity();
        assert(params.dt >= ZERO && max_lin >= ZERO && max_corrective >= ZERO, errors::NEGATIVE);
        // π/4 per FULL frame, exactly upstream's MAX_ROTATION policy.
        let max_ang = Fixed { raw: 3373259426 } * params.inv_dt();
        FreeBodySolver {
            gravity,
            full_dt: params.dt,
            dt,
            max_lin,
            max_ang,
            iterations: params.num_solver_iterations,
        }
    }

    /// `rb` after `SolverBodyStoreTrait::from_bodies`, `solve_island` and `to_bodies`, when no
    /// manifold and no enabled joint of the step references `handle`: per substep the force
    /// increment then the capped integration, then full-step damping, then velocities and
    /// `next_position` written back (none for a fixed or disabled body). Bit-identical to the
    /// store path: same expressions in the same order (products floor, divisions round to
    /// nearest, rotations renormalize). Zero dt leaves the velocities and pose unintegrated.
    /// The default 4 substeps are unrolled (−28k gas per body against the loop,
    /// `free_alternatives::solve_looped`); other counts loop. Inlined: the body is not copied
    /// into a call.
    #[inline(always)]
    fn solve(self: @FreeBodySolver, handle: Handle, rb: RigidBody) -> RigidBody {
        let (mut b, step) = gather(handle, rb, *self.gravity, *self.dt);
        let mut rb = rb;
        if !step.moving {
            return rb;
        }
        let full_dt = *self.full_dt;
        if full_dt != ZERO {
            let (dt, max_lin, max_ang) = (*self.dt, *self.max_lin, *self.max_ang);
            let iterations = *self.iterations;
            if iterations == 4 {
                add_force(ref b, step.increment);
                integrate_body(ref b, dt, max_lin, max_ang);
                add_force(ref b, step.increment);
                integrate_body(ref b, dt, max_lin, max_ang);
                add_force(ref b, step.increment);
                integrate_body(ref b, dt, max_lin, max_ang);
                add_force(ref b, step.increment);
                integrate_body(ref b, dt, max_lin, max_ang);
            } else {
                let mut substep = 0;
                while substep != iterations {
                    add_force(ref b, step.increment);
                    integrate_body(ref b, dt, max_lin, max_ang);
                    substep += 1;
                }
            }
            damp_body(ref b, step.damping, full_dt);
        }
        writeback(b, step, ref rb);
        rb
    }
}

#[cfg(test)]
mod benches;

#[cfg(test)]
mod fixtures;

#[cfg(test)]
mod free_alternatives;

#[cfg(test)]
mod free_checks;

#[cfg(test)]
mod idle_benches;

#[cfg(test)]
mod tests;

# DA — `rapier_dynamics2d::rigid_body`: vector-valued body components and integration

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D3 note on inverse inertia, D8, D9); `docs/research/01-rapier-analysis.md`
§3.2, §4.1–4.5; `crates/rapier_core/src/rigid_body/*.cairo` (scalar components already ported:
`RigidBodyType`, `RigidBodyDamping::damping_factor`, `RigidBodyActivation`, `RigidBodyDominance`,
`RigidBodyChanges`), `crates/rapier_core/src/integration_parameters.cairo`,
`crates/rapier_geometry2d/src/mass.cairo` (`MassProperties`), `crates/rapier_math/src/{rot2,pose2}.cairo`
(M2). Upstream (`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/rapier/src/dynamics/rigid_body_components.rs` (`RigidBodyPosition`, `RigidBodyVelocity`,
`RigidBodyMassProps`, `RigidBodyForces`, `RigidBodyAdditionalMassProps`, `integrate_forces`,
`integrate`, `apply_damping`, `world_inv_inertia`, `velocity_at_point`, `kinetic_energy`,
`gravity_scale`, locked axes / `LockedAxes`), `$UP/rapier/src/dynamics/rigid_body.rs` (public
`RigidBody` methods that only touch these components: `apply_impulse`, `apply_torque_impulse`,
`apply_impulse_at_point`, `add_force`, `reset_forces`, `set_translation/rotation/position`,
`predict_position_using_velocity_and_forces`). Golden vectors: `rapier_golden::scenes` ball_drop
free-fall samples (`4` substeps) and `integration_parameters`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/rigid_body.cairo` + `src/rigid_body/*.cairo`,
`crates/rapier_dynamics2d/tests/rigid_body_golden.cairo`, `gas/rapier_dynamics2d/rigid_body.snap`,
`gas/rapier_dynamics2d_integrationtest/rigid_body_golden.snap`. `lib.cairo` already declares
`pub mod rigid_body;`. Do not touch `rapier_core`, `rapier_geometry2d`, or any `Scarb.toml`.

## 3. Expected API and semantics
```cairo
pub struct RigidBodyPosition { pub position: Pose2, pub next_position: Pose2 }
pub struct RigidBodyVelocity { pub linvel: Vec2, pub angvel: Fixed }
pub struct RigidBodyMassProps { pub flags: LockedAxes, pub local_mprops: MassProperties,
    pub world_com: Vec2, pub effective_inv_mass: Vec2, pub effective_world_inv_inertia: Fixed }
pub struct RigidBodyForces { pub force: Vec2, pub torque: Fixed, pub gravity_scale: Fixed, pub user_force: Vec2, pub user_torque: Fixed }
pub struct LockedAxes { pub bits: u8 }   // TRANSLATION_LOCKED_X/Y, ROTATION_LOCKED
```
Methods with upstream names: `RigidBodyVelocity::{zero, integrate(dt, position, local_com) -> Pose2,
apply_damping(dt, damping) (uses rapier_core damping_factor), velocity_at_point, kinetic_energy,
pseudo_kinetic_energy, is_zero, apply_impulse(mprops, impulse), apply_torque_impulse,
apply_impulse_at_point}`, `RigidBodyForces::{integrate(dt, velocity, mprops) -> RigidBodyVelocity,
add_linear_acceleration, gravity_force}`, `RigidBodyMassProps::{update_world_mass_properties(position),
effective_mass, effective_angular_inertia, from_local(mprops, flags)}`,
`RigidBodyPosition::{integrate_forces_and_velocities(dt, forces, vels, mprops) -> RigidBodyPosition,
pose_error / interpolate if present}`; all rotations advanced via `Rot2::integrate(angvel, dt)`
(M2) with the renormalisation policy M2 recommends (read its `rot2.cairo` docs). `inv(0) = 0`
everywhere a fixed body appears. DEFER: gyroscopic terms (3D), CCD fields, dominance (already in
core), user data, `RigidBodySet` (DD).

## 4. Efficiency and variants
`integrate` and `apply_impulse_at_point` run per body per substep: fused kernels (`gcross`,
`dot2_add`), no division (inverse masses precomputed in `update_world_mass_properties`). Bench
`integrate` as (a) direct port and (b) with the translation/rotation update fused; report the
per-body cost.

## 5. Tests
Golden: ball_drop free fall replayed with 4 substeps of `dt/4` matching `rapier_golden::scenes`
samples (before contact) within the README tolerance; damping values from `rapier_core` tests;
impulse at a point changes `angvel` by `gcross(r, impulse)·inv_inertia`; locked axes zero the
corresponding components; kinetic energy of a moving body. Table-driven, ≤ 800 lines per file,
`fuzz_*` ≤ 4, `gas_*` for every public function and candidate.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_dynamics2d::rigid_body`, `… rapier_dynamics2d_integrationtest::rigid_body_golden`; conventional
commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.

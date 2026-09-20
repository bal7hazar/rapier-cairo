# DB — `rapier_dynamics2d::collider`: vector-valued collider components and builder

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D6, D9); `docs/research/01-rapier-analysis.md` §3.3;
`crates/rapier_core/src/collider/*.cairo` (scalar components already ported: `ColliderType`,
`ColliderMaterial`, `ColliderFlags`, `ActiveEvents`, `ActiveHooks`, `ActiveCollisionTypes`,
`ColliderChanges`), `crates/rapier_core/src/interaction_groups.cairo`,
`crates/rapier_geometry2d/src/shape.cairo` (GB: `Shape`, `ShapeTrait`), `mass.cairo`,
`crates/rapier_math/src/pose2.cairo` (M2). Upstream
(`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/rapier/src/geometry/collider_components.rs` (`ColliderParent`, `ColliderPosition`,
`ColliderShape`, `ColliderMassProps`, `ColliderBroadPhaseData`), `$UP/rapier/src/geometry/collider.rs`
(`Collider`, `ColliderBuilder` and its setters, `mass_properties`, `compute_aabb`,
`set_position_wrt_parent`, `parent`, `is_sensor`). Golden vectors: `rapier_golden::scenes` bodies
(collider descriptions: shape, pose wrt parent, density, friction, restitution) and
`rapier_golden::aabb`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/collider.cairo` + `src/collider/*.cairo`,
`crates/rapier_dynamics2d/tests/collider_golden.cairo`, `gas/rapier_dynamics2d/collider.snap`,
`gas/rapier_dynamics2d_integrationtest/collider_golden.snap`. `lib.cairo` already declares
`pub mod collider;`. Do not touch `rapier_core`, `rapier_geometry2d`, any `Scarb.toml`.

## 3. Expected API and semantics
```cairo
pub struct ColliderParent { pub handle: Handle, pub pos_wrt_parent: Pose2 }
pub struct ColliderPosition { pub pose: Pose2 }
pub enum ColliderMassProps { Density: Fixed, Mass: Fixed, MassProperties: MassProperties }
pub struct Collider { pub co_type: ColliderType, pub shape: Shape, pub mprops: ColliderMassProps,
    pub changes: ColliderChanges, pub parent: Option<ColliderParent>, pub pos: ColliderPosition,
    pub material: ColliderMaterial, pub flags: ColliderFlags, pub contact_force_event_threshold: Fixed,
    pub user_data: u128 }
```
`ColliderTrait` with upstream names: `is_sensor`, `set_sensor`, `parent`, `position`,
`translation`, `rotation`, `set_position`, `position_wrt_parent`, `set_position_wrt_parent`,
`shape`, `set_shape`, `density`, `mass`, `mass_properties()` (resolves the three `ColliderMassProps`
variants through `Shape::mass_properties`), `compute_aabb()`, `compute_swept_aabb` (DEFER, CCD),
`friction`, `restitution`, `set_friction`, `set_restitution`, `collision_groups`,
`set_collision_groups`, `solver_groups`, `active_events`, `active_hooks`, `active_collision_types`,
`set_enabled`, `is_enabled`, `contact_force_event_threshold`. `ColliderBuilder` with `new(shape)`,
`ball(r)`, `cuboid(hx, hy)`, `capsule_y(half_height, r)`, `capsule_x`, `segment(a, b)`,
`halfspace(n)`, `density`, `mass`, `mass_properties`, `friction`, `restitution`,
`friction_combine_rule`, `restitution_combine_rule`, `sensor`, `translation`, `rotation`,
`position`, `collision_groups`, `solver_groups`, `active_events`, `active_hooks`,
`active_collision_types`, `enabled`, `user_data`, `contact_force_event_threshold`, `build()` with
upstream defaults (density 1, friction 0.5, restitution 0, not sensor, enabled). DEFER:
`ColliderSet` (DD), CCD fields, contact skin.

## 4. Efficiency and variants
Builder and getters are cold paths: keep them simple. `compute_aabb` delegates to
`Shape::compute_aabb(pose)`; `mass_properties` computes the density variant with one fused
kernel. No candidates required unless a getter is non-trivial.

## 5. Tests
Golden: rebuild every collider of `rapier_golden::scenes` through the builder and compare
`mass_properties()` and `compute_aabb()` (pose = parent pose × pose wrt parent) with
`rapier_golden::mass_properties` / `aabb` where a case matches, else with the analytic value;
table-driven builder defaults and setters; sensor/enabled flags; `fuzz_*` ≤ 4; `gas_*` for
`build`, `mass_properties`, `compute_aabb`. ≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_dynamics2d::collider`, `… rapier_dynamics2d_integrationtest::collider_golden`; conventional
commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.

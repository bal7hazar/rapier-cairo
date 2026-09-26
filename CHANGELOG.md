# Changelog

All crates of the workspace share one version. Alphas carry no API or numeric stability guarantee; every entry says
whether simulation results changed.

## 0.1.0-alpha.2 — 2026-09-26

**Results:** bit-identical to `0.1.0-alpha.1` on every golden vector and scene test (the level impact windows are
pinned by state digests), except for the two upstream-alignment changes listed under *Changed*.

### Breaking
- `WorldState` is **version 2**: it now carries the persistent active set. `WorldTrait::from_state` panics with
  `'world state: version'` on a version-1 state; save states again with this version (#143).
- `rapier2d::pipeline::sleeping::wake_removed_partners` is removed (#135).

### Performance (Cairo steps)
- Awake contact tick: split contact sweeps, −41.0 % / −37.5 % on the level-10 / level-20 impact windows, −32 % to
  −46 % on the contact benchmark scenes (#139).
- Sleeping bodies cost nothing per tick: a persistent active set walks only the awake bodies; the all-asleep engine step
  of a 10-block level goes from 36,141 to 1,000 steps, its flight tick from 60,230 to 16,489 (#143). World loading
  +4–5 %, the small benchmark scenes +1–3 %.

### Added
- `WorldTrait::{is_sleeping, linvel, angvel}`: activation and velocity reads without copying the body (#143).
- Collider API and world facade (#135): `ColliderTrait::{compute_collision_aabb, compute_broad_phase_aabb,
  compute_swept_aabb, copy_from}`, `ColliderBuilderTrait::{convex_hull, position_wrt_parent, delta, default_density,
  default_friction}`, `ColliderSetTrait::{set_parent, iter_enabled, with_capacity, invalid_handle, get_pair_mut}`,
  `WorldTrait::{active_bodies, num_active_bodies, wake_up_all, rigid_bodies, all_colliders}`, `CollisionPipeline`
  (broad and narrow phase and events without the solver), `PhysicsWorld` alias; prelude re-exports of `PhysicsWorld`,
  `CollisionPipeline(Trait)`, `PhysicsPipeline(Trait)`, and `ColliderPair(Trait)` in `rapier_geometry2d` (#137).

### Changed (upstream alignment)
- A body built `.sleeping(true)` stays asleep when it is inserted: a collider inserted since the last step wakes
  nobody, as in rapier-rs (a pile whose bodies touch still wakes at its first step through contact start, as upstream)
  (#143).
- Removing a collider ends its pairs at the next step with `Stopped | REMOVED` (`| SENSOR`) without waking its sensor
  partners, as upstream (#135).

## 0.1.0-alpha.1 — 2026-09-25

First publication on scarbs.xyz: `rapier_math`, `rapier_core`, `rapier_geometry2d`, `rapier_dynamics2d`, `rapier2d`
(#134). 2D rigid bodies (dynamic, fixed, kinematic), colliders (ball, cuboid, capsule, segment, half-space, convex
polygon), sensors and collision / contact-force events, one-way platforms, fixed / revolute / prismatic / rope /
spring joints with limits and motors, the soft-contact substep solver, sleeping, ray / point / AABB queries, and a
versioned `WorldState` save / restore. Validated against golden vectors from rapier2d-f64 0.35.3 / parry2d-f64 0.30.2;
deliberate divergences in `docs/adr/0001-upstream-divergences.md`.

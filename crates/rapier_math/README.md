# rapier_math

Physics-side math of rapier-cairo on top of glam-cairo's Q32.32 `Vec2`: `Rot2`, `Pose2`, fused kernels and scalar helpers.

Part of [rapier-cairo](https://github.com/bal7hazar/rapier-cairo), a port of the [Rapier](https://rapier.rs)
physics engine to Cairo for provable physics. Q32.32 fixed-point scalars from
[fixed-cairo](https://github.com/bal7hazar/fixed-cairo) and vectors from [glam-cairo](https://github.com/bal7hazar/glam-cairo).

Most users depend on `rapier2d`.

## Stability

`0.1.0-alpha.1` is an **alpha**: no API or numeric stability. Every alpha is validated against golden vectors
recorded from rapier2d-f64 0.35.3 / parry2d-f64 0.30.2 within documented tolerance bands, and
the deliberate divergences from upstream are listed in
[`docs/adr/0001-upstream-divergences.md`](https://github.com/bal7hazar/rapier-cairo/blob/main/docs/adr/0001-upstream-divergences.md).
From `0.1.0` on, the siblings' versioning policy applies: a numeric change is a MINOR bump, so pinning a version
pins the simulation's results.

## License

MIT

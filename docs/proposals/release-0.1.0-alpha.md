# Release assessment — `0.1.0-alpha.1` of the rapier-cairo crates on scarbs.xyz

Status: **done** — `0.1.0-alpha.1` published on 2026-09-25 (owner's go; #134, tag `v0.1.0-alpha.1`). Choices made: option
(b) for the dev-dependencies (`rapier_testing` / `rapier_golden` unpublished); `examples/ball_drop` stays on a path
dependency so that the `execute` CI job tracks `main` (games pin the registry version).
Requested by the project-manager session (2026-09-25): the game must consume `rapier2d` by registry version.

## What `scarb package` says today (measured 2026-09-25, scarb 2.19.4)

`scarb package -p rapier_core --no-verify` fails: *"dependency `rapier_golden` does not specify a version
requirement — all dependencies must have a version specified when packaging; the `path` specification will be
removed"*, and warns *"manifest has no readme"*. The rule applies to **dev-dependencies too**: every crate
dev-depends on `rapier_testing` (and most on `rapier_golden`) by path only.

## What the release needs (one PR when the owner says go)

1. **Versions on every intra-workspace dependency**: `rapier_core = { path = "../rapier_core", version =
   "0.1.0-alpha.1" }` and so on; the workspace `version` becomes `0.1.0-alpha.1` (pre-release tags sort below
   `0.1.0` under semver, so a later `0.1.0` supersedes them).
2. **Dev-dependencies**: either (a) publish `rapier_testing` (25 lines) and `rapier_golden` (28 k lines of generated
   fixtures) as `0.1.0-alpha.1` too — simplest, but ships test fixtures to the registry — or (b) have the release
   script package from a temporary copy whose manifests drop `[dev-dependencies]` (tests are not needed by
   consumers). Recommendation: (b) for `rapier_golden`, (a) for `rapier_testing` only if a consumer wants `opaque`.
3. **Publication order** (each needs the previous ones on the registry): `rapier_math` → `rapier_core` →
   `rapier_geometry2d` → `rapier_dynamics2d` → `rapier2d`; external deps already come from the registry (`fixed`,
   `glam` 0.3.0). `examples/ball_drop` switches to `rapier2d = "0.1.0-alpha.1"` in the same PR.
4. **Metadata per crate**: `readme` (a short README per crate), `repository` (already inherited:
   `https://github.com/bal7hazar/rapier-cairo`), `license` (inherited, MIT), `description` (present), `keywords`,
   `documentation` (link to the repo docs).
5. **A release script** `scripts/release.sh <version>` (orchestrator-owned): checks a clean `main`, the full gate,
   `gas.py check`, `api_parity.py --check`; bumps the workspace version; packages each crate in order with
   `scarb package` (+ `scarb publish` behind an explicit `--publish` flag and the owner's scarbs.xyz token, which
   only the owner enters); tags `v<version>`. glam-cairo's release precedent (tag-driven) is the model.
6. **API stability note** in the README: alpha = no API or numeric stability; the versioning policy of the siblings
   applies from 0.1.0 (a numeric change is a MINOR bump; pinning the version pins the golden vectors).

## Open points for the owner

Crate names on the registry (`rapier2d` may collide with nothing on scarbs.xyz, but check), whether to publish the
geometry crate under a parry-like name now or after the parry split (`docs/proposals/parry-split.md`), and who holds
the publishing token.

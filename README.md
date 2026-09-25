# rapier-cairo

A port of the [Rapier](https://rapier.rs) physics engine to [Cairo](https://www.cairo-lang.org),
so that games can run on **provable physics**.

> Status: foundations. The research is done and the execution plan is set; the engine itself is next.

- [`docs/PLAN.md`](docs/PLAN.md) — decisions, architecture, phases and work packages
- [`docs/research/`](docs/research) — deep dives into Rapier and Parry, a review of the Cairo
  ecosystem (alexandria, origami, starknet-agentic) and a 415-probe numeric benchmark (cubit, Orion,
  hand-written fixed-point representations)
- [`benchmarks/numeric/`](benchmarks/numeric) — the reproducible benchmark behind the scalar choice
- [`AGENTS.md`](AGENTS.md) — how the project is developed (orchestrator + parallel executors)

## Design in one paragraph

Signed Q32.32 fixed point in a native `i64`, fused multiply-accumulate kernels (one rescale per
output), 2D first, closed shape enum, stateless broad phase, Rapier's substepped soft-constraint
solver run sequentially in a documented order, pure-Cairo core crates, and golden vectors generated
from the Rust engine to validate every layer.

## Gas is tracked per test

Every unit test is a gas probe. `scripts/gas.py` records the Sierra gas of each test in
`gas/<crate>/<module>.snap`; CI fails on any drift, so the snapshot diff of a PR *is* its gas report. When the
cheapest implementation is not obvious (arithmetic vs bitwise vs loop), all candidates are
implemented, measured, and the losers stay in the tree to be re-evaluated on compiler upgrades.

```bash
snforge test --workspace
```

```bash
python3 scripts/gas.py diff
```

Toolchain: scarb 2.19.4, starknet-foundry 0.61.0 (see `.tool-versions`).

## License

MIT

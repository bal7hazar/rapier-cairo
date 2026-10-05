# Packages

Generated file, do not edit by hand. Run: 2026-10-05, runner: ubuntu-latest, 4 vCPU / 15 GB, 3 cold build round(s) per consumer (medians of the per-round differences). Regenerate: run `python3 scripts/consumer_cost.py --repeat 5 --interleave --json consumer_cost.json` (CI: the `Consumer cost` job uploads `consumer_cost.json` and this table as the artifact `consumer-cost`), then `python3 scripts/packages_table.py consumer_cost.json --runner RUNNER --output docs/PACKAGES.md`.

Gates: at most 40,000 library lines; marginal cost at most 5 s / 1 GB (gate 2); closures 15 s / 3 GB unless they declare a budget (gate 3). Each figure is `value / limit (margin)`; the margin is the room left below the limit, negative when over it.

## Published packages

| package | version | lines | marginal time | marginal memory | verdict |
|---|---|---:|---:|---:|---|
| rapier2d | 0.1.0-alpha.10 | 10,565 (gate 3 only) | closure 7.8 s / 15 s (+48 %) | closure 1.51 GB / 3 GB (+50 %) | ok |
| rapier2d_classes | 0.1.0-alpha.10 | 3,760 / 40,000 (+91 %) | 0.2 s / 5 s (+95 %) | 0.10 GB / 1 GB (+90 %) | ok |
| rapier_core | 0.1.0-alpha.10 | 3,200 / 40,000 (+92 %) | 0.5 s / 5 s (+89 %) | 0.08 GB / 1 GB (+92 %) | ok |
| rapier_dynamics2d | 0.1.0-alpha.10 | 15,333 / 40,000 (+62 %) | 1.8 s / 5 s (+65 %) | 0.36 GB / 1 GB (+64 %) | ok |
| rapier_geometry2d | 0.1.0-alpha.10 | 25,128 / 40,000 (+37 %) | 2.2 s / 5 s (+57 %) | 0.46 GB / 1 GB (+54 %) | ok |
| rapier_math | 0.1.0-alpha.10 | 935 / 40,000 (+98 %) | 0.3 s / 5 s (+94 %) | 0.02 GB / 1 GB (+98 %) | ok |

## Declared closures

| closure | members | time | memory | budget | verdict |
|---|---|---:|---:|---|---|
| game_classes | rapier2d_classes, glam_core@0.5.0, fixed@0.5.0 | 8.0 s / 15 s (+47 %) | 1.61 GB / 3 GB (+46 %) | 15 s / 3 GB | ok |
| rapier2d | rapier2d, glam_core@0.5.0, fixed@0.5.0 | 7.5 s / 15 s (+50 %) | 1.51 GB / 3 GB (+50 %) | 15 s / 3 GB | ok |

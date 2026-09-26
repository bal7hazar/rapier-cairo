# rapier_sink

Starknet contract fixtures that link the `rapier2d` step into deployable classes, so that the
compiled class size of a game-like consumer is tracked against the network limits (work package
CS1). **Not published** (`scripts/release.sh` lists the published crates).

`python3 scripts/bytecode_size.py [table|snapshot|check|attribution]` builds this package in the
release profile and measures every class; `gas/bytecode.size` is the committed snapshot (checked by
CI's `bytecode` job). The analysis is in `docs/research/class-size.md`.

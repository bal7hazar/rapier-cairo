# rapier_sink

Starknet contract fixtures that link the `rapier2d` step into deployable classes, so that the
compiled class size of a game-like consumer is tracked against the network limits (work package
CS1). **Not published** (`scripts/release.sh` lists the published crates).

`python3 scripts/bytecode_size.py [table|snapshot|check|attribution]` builds this package in the
release profile and measures every class; `gas/bytecode.size` is the committed snapshot (checked by
CI's `bytecode` job). The analysis is in `docs/research/class-size.md`.

`programs/lib.cairo` holds the `#[executable]` fixtures (CS2): the game world stepped under each
step configuration (`rapier2d::pipeline::config`). It is not part of this package:
`scripts/bytecode_size.py` builds it as a temporary executable package and records each
program's felts (`program.*` in `gas/bytecode.size`), what a proof of the game hashes.

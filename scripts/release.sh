#!/usr/bin/env bash
# Release the rapier-cairo crates on scarbs.xyz (docs/proposals/release-0.1.0-alpha.md).
#
#   scripts/release.sh bump <version>   rewrite workspace.package.version and the intra-workspace
#                                       requirements in the root Scarb.toml (commit it in a release PR)
#   scripts/release.sh package          dry run: package every published crate (--no-verify)
#   scripts/release.sh publish          publish in dependency order, then tag v<version> and push the tag
#
# Published, in this order: rapier_math, rapier_core, rapier_geometry2d, rapier_dynamics2d, rapier2d.
# rapier_testing and rapier_golden (test helpers, 28k lines of fixtures) are not published: the crates
# are packaged from a staged copy of HEAD whose manifests drop [dev-dependencies] (tests are not part
# of what consumers build). `publish` needs a clean checkout of origin/main and SCARB_REGISTRY_AUTH_TOKEN
# (the owner's scarbs.xyz token); each crate is verified against the registry copies of the previous
# ones, so a crate is retried while the registry index catches up.
set -euo pipefail

CRATES=(rapier_math rapier_core rapier_geometry2d rapier_dynamics2d rapier2d)
ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

version() { sed -n '/^\[workspace.package\]/,/^\[/{s/^version = "\(.*\)"$/\1/p}' Scarb.toml; }

stage() {
  STAGE="$(mktemp -d)"
  git archive HEAD | tar -x -C "$STAGE"
  for c in "${CRATES[@]}"; do
    # Drop the [dev-dependencies] table (up to the next table header).
    sed -i '/^\[dev-dependencies\]/,/^\[/{/^\[dev-dependencies\]/d;/^\[/!d}' "$STAGE/crates/$c/Scarb.toml"
  done
  echo "$STAGE"
}

case "${1:-}" in
  bump)
    NEW="${2:?usage: release.sh bump <version>}"
    OLD="$(version)"
    sed -i "/^\[workspace.package\]/,/^\[/s/^version = \"$OLD\"$/version = \"$NEW\"/" Scarb.toml
    sed -i -E "s/^(rapier_[a-z0-9]+ = \{ path = \"crates\/rapier_[a-z0-9]+\", version = )\"$OLD\"/\1\"$NEW\"/" Scarb.toml
    echo "workspace $OLD -> $(version)"; grep -n "version = \"$NEW\"" Scarb.toml
    ;;
  package)
    STAGE="$(stage)"
    for c in "${CRATES[@]}"; do (cd "$STAGE" && scarb package -p "$c" --no-verify --allow-dirty); done
    ls -la "$STAGE/target/package"
    echo "staged in $STAGE (dry run, nothing published)"
    ;;
  publish)
    V="$(version)"
    [ -n "${SCARB_REGISTRY_AUTH_TOKEN:-}" ] || { echo "SCARB_REGISTRY_AUTH_TOKEN is not set" >&2; exit 64; }
    git fetch -q origin
    [ -z "$(git status --porcelain)" ] || { echo "working tree not clean" >&2; exit 65; }
    [ "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)" ] || { echo "HEAD is not origin/main" >&2; exit 65; }
    STAGE="$(stage)"
    for c in "${CRATES[@]}"; do
      for attempt in 1 2 3 4 5 6; do
        if (cd "$STAGE" && scarb publish -p "$c" --allow-dirty); then break; fi
        [ "$attempt" = 6 ] && { echo "publishing $c failed" >&2; exit 1; }
        echo "retrying $c in 30 s (registry index)"; sleep 30
      done
    done
    git tag -a "v$V" -m "rapier-cairo $V"
    git push -q origin "v$V"
    echo "published ${CRATES[*]} at $V, tagged v$V"
    ;;
  *) sed -n '2,15p' "$0"; exit 64 ;;
esac

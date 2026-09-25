#!/usr/bin/env bash
# Build locks shared by the orchestrators of this machine (31 GB, no swap):
#   * one PROJECT lock per repository: at most one Cairo build/test of rapier-cairo at a time;
#   * the machine-wide HEAVY lock (`~/orchestrator/heavy-build.lock`, shared with glam-cairo and
#     nalgebra-cairo) for workspace-wide test runs, which peak near 10 GB.
# A crate-scoped build or test (`snforge test -p <crate>`, `scarb build/lint`) only takes the project
# lock, so it runs next to another project's heavy build. Lock order is always project -> heavy (no
# cycle: the other projects only take the heavy lock). Nested calls (snforge -> scarb) inherit the locks.
#
#   lock.sh <real-binary> <args...>
set -u
real="$1"; shift
project_lock="${RAPIER_PROJECT_LOCK:-$HOME/orchestrator/locks/rapier-cairo.lock}"
heavy_lock="${HEAVY_BUILD_LOCK:-$HOME/orchestrator/heavy-build.lock}"
mkdir -p "$(dirname "$project_lock")"
[ -n "${RAPIER_BUILD_LOCK_HELD:-}" ] && exec "$real" "$@"
heavy=0
case "$(basename "$real")" in
  snforge)
    [ "${1:-}" = test ] || exec "$real" "$@"
    heavy=1
    for a in "$@"; do case "$a" in -p|--package|--package=*|-p*) heavy=0 ;; esac; done
    for a in "$@"; do [ "$a" = --workspace ] && heavy=1; done ;;
  scarb)
    case "${1:-}" in
      build|lint|check|test|execute) ;;
      prove|verify) heavy=1 ;;
      *) exec "$real" "$@" ;;
    esac
    for a in "$@"; do [ "$a" = --workspace ] && [ "${1:-}" = test ] && heavy=1; done ;;
  *) exec "$real" "$@" ;;
esac
# Same CPU policy as the machine-wide shim in ~/.local/bin (8 vCPU: a sustained 100 % makes the
# hypervisor throttle the VM): capped build parallelism, lowered priority.
export RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-4}" CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-4}"
export RAPIER_BUILD_LOCK_HELD=1 HEAVY_BUILD_LOCK_HELD=1
if [ "$heavy" = 1 ]; then
  exec nice -n 10 flock "$project_lock" flock "$heavy_lock" "$real" "$@"
fi
exec nice -n 10 flock "$project_lock" "$real" "$@"

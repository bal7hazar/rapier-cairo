#!/usr/bin/env bash
# Run the `examples/ball_drop` executable: execute, then (PROVE=1 only) prove and verify.
#
#   scripts/prove-example.sh [scene] [steps]
#   PROVE=1 scripts/prove-example.sh box_stack3 10
#
#   scene  ball_drop (default, 0) | box_stack3 (1) | pendulum (2)
#   steps  number of World::step calls (default 10)
#
# Stages, each timed; the script exits non-zero as soon as one fails:
#   build    always: `scarb build` (kept out of the execute time)
#   execute  always: `scarb execute --print-program-output --print-resource-usage`
#   prove    PROVE=1: `scarb prove` on the execution of the first stage (Stwo prover)
#   verify   PROVE=1: `scarb verify` on that proof
#
# The prover has a memory floor that a shared 31 GB machine cannot give (see the example's
# README): prove and verify are opt-in. Both run behind the machine-wide heavy-build lock
# (`$HEAVY_BUILD_LOCK`, default `$HOME/orchestrator/heavy-build.lock`) when its directory exists.
set -euo pipefail

SCENE="${1:-ball_drop}"
STEPS="${2:-10}"
case "$SCENE" in
  ball_drop|0) SCENE=0 ;;
  box_stack3|1) SCENE=1 ;;
  pendulum|2) SCENE=2 ;;
  *) echo "unknown scene '$SCENE' (ball_drop | box_stack3 | pendulum)" >&2; exit 64 ;;
esac
case "$STEPS" in ''|*[!0-9]*) echo "steps must be a non-negative integer" >&2; exit 64 ;; esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/examples/ball_drop"
ARGS="$SCENE,$STEPS"

LOCK="${HEAVY_BUILD_LOCK:-$HOME/orchestrator/heavy-build.lock}"
locked() {
  if command -v flock >/dev/null && [ -d "$(dirname "$LOCK")" ]; then
    flock "$LOCK" "$@"
  else
    "$@"
  fi
}

timed() { # timed <stage> <command...>: run, print the wall time, keep the exit status
  local stage="$1" start end status=0
  shift
  start=$(date +%s%N)
  "$@" || status=$?
  end=$(date +%s%N)
  printf '[%s] %s in %d.%03ds (exit %d)\n' "$stage" "$([ $status -eq 0 ] && echo ok || echo FAILED)" \
    $(((end - start) / 1000000000)) $(((end - start) / 1000000 % 1000)) "$status"
  return $status
}

LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT
echo "== build =="
timed build scarb build || exit 1
echo "== execute: scene=$SCENE steps=$STEPS =="
timed execute scarb execute --no-build --arguments "$ARGS" --output standard \
  --print-program-output --print-resource-usage 2>&1 | tee "$LOG"
[ "${PIPESTATUS[0]}" -eq 0 ] || exit 1

[ "${PROVE:-0}" = "1" ] || { echo "prove and verify skipped (set PROVE=1)"; exit 0; }

EXEC_ID="$(sed -n 's|^Saving output to: .*/execution\([0-9]*\)$|\1|p' "$LOG" | tail -n 1)"
[ -n "$EXEC_ID" ] || { echo "cannot find the execution id in the execute output" >&2; exit 1; }

echo "== prove: execution $EXEC_ID =="
timed prove locked scarb prove --execution-id "$EXEC_ID" || exit 1
PROOF="target/execute/ball_drop/execution$EXEC_ID/proof.json"
[ ! -f "$PROOF" ] || echo "proof size: $(wc -c <"$PROOF") bytes ($PROOF)"

echo "== verify: execution $EXEC_ID =="
timed verify locked scarb verify --execution-id "$EXEC_ID" || exit 1

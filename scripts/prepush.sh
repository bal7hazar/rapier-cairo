#!/usr/bin/env bash
# Pre-push check: what CI would reject, caught locally in seconds to minutes (lot PP1, docs/briefs/pp1-prepush.md).
#
#   scripts/prepush.sh            # from any directory of a clone or worktree
#   PREPUSH_FULL=1 scripts/prepush.sh   # stages 3-4 on everything (manual check)
#
# What changed = the branch against `git merge-base HEAD origin/main`, plus the working tree and untracked files
# (no fetch: the local `origin/main` is used; without it everything counts as changed).
#
#   1. fmt        scarb fmt --check --workspace                 always
#   2. scripts    --self-test of the Python scripts that have one   always (a second or two)
#   3. build/lint scarb build / lint -p <crate>                 each crate with a changed file; the whole workspace
#                                                              when a root manifest, Scarb.lock or .tool-versions changed
#   4. artefacts  only when their inputs changed, each with its trigger:
#                 api-parity    crates/*/src/**/*.cairo, docs/API_PARITY.md, scripts/api_parity.py   -> runs --check
#                 gas           gas/**/*.snap                                -> prints the `gas.py check` command (needs the tests)
#                 bytecode.size class sources, manifests, toolchain         -> warns (generated in CI only)
#                 PACKAGES.md   crate sources, manifests, consumer_cost.*   -> warns (generated in CI only)
#
# It checks HEAD plus the working tree, not the refs being pushed (an uncommitted fix can let a red commit pass), and
# compiles only the touched crates: CI catches their dependents. `scarb` on PATH must be the build shim (a warning says
# when it is not). The heavy-lock wait is capped at 90 s (PREPUSH_LOCK_WAIT_MAX), then the compile is left to CI.
#
# Never `snforge test --workspace`, never a whole-shot suite (`rapier2d_classes`, `rapier_sink`): those stay in CI.
# Builds go through the build shims (heavy-build.lock). Every build runs with RAYON_NUM_THREADS=1. Exits non-zero on the first failing stage and names it.
set -euo pipefail

export RAYON_NUM_THREADS=1
cd "$(git rev-parse --show-toplevel)"

start=$SECONDS
stage_name=""
stage_start=$SECONDS

fail() {
  echo "prepush: FAILED at ${stage_name:-setup}" >&2
  exit 1
}
trap fail ERR

begin() {
  stage_name="$1"
  stage_start=$SECONDS
  echo "==> $stage_name"
}

end() {
  echo "    $stage_name: $((SECONDS - stage_start)) s"
}

note() { echo "    $*"; }

# Where the VPS heavy-build lock exists (and `scarb` on PATH is its shim) a compile may queue behind other projects'
# builds: wait for it at most PREPUSH_LOCK_WAIT_MAX seconds (90), always through the shim, never around it. If the
# lock is not obtained in time, the compile is skipped (one line) and left to CI; the stage still passes. Where there
# is no such lock (the Mac: no ~/orchestrator/heavy-build.lock, or no shim) the compile always runs.
lock_max="${PREPUSH_LOCK_WAIT_MAX:-90}"
shim_lock="${HEAVY_BUILD_LOCK:-$HOME/orchestrator/heavy-build.lock}"
scarb_bin=$(command -v scarb || true)
capped=0
if [ -e "$shim_lock" ]; then
  if [ -n "$scarb_bin" ] && grep -qs 'HEAVY_BUILD_LOCK\|heavy-build\.lock' "$scarb_bin"; then
    capped=1
  else
    echo "prepush: WARNING $shim_lock exists but \`scarb\` on PATH (${scarb_bin:-none}) is not the shim that takes it: builds run unlocked" >&2
  fi
fi
compile_skipped=0
lock_wait_total=0

# locked_run <cmd...>: run a build command (the shim on PATH takes the lock before it execs scarb, so the first output
# line marks the moment the lock was acquired). The reported "lock wait" is an upper bound: lock wait plus scarb
# start-up.
locked_run() {
  [ "$compile_skipped" = 1 ] && return 0
  local t0=$SECONDS first="" out rcf pid rc
  out=$(mktemp); rcf=$(mktemp)
  setsid bash -c 'o=$0; r=$1; shift; "$@" >"$o" 2>&1; echo $? >"$r"' "$out" "$rcf" "$@" &
  pid=$!
  while [ ! -s "$rcf" ]; do
    if [ -z "$first" ] && [ -s "$out" ]; then
      first=$((SECONDS - t0))
      lock_wait_total=$((lock_wait_total + first))
      note "lock wait (upper bound): $first s ($*)"
    fi
    if [ -z "$first" ] && [ "$capped" = 1 ] && [ $((SECONDS - t0)) -ge "$lock_max" ] && [ ! -s "$rcf" ]; then
      kill -- "-$pid" 2>/dev/null || true
      lock_wait_total=$((lock_wait_total + SECONDS - t0))
      compile_skipped=1
      rm -f "$out" "$rcf"
      echo "heavy lock busy: Cairo compile left to CI"
      return 0
    fi
    sleep 1
  done
  wait "$pid" 2>/dev/null || true
  sed 's/^/    /' "$out"
  rc=$(cat "$rcf"); rm -f "$out" "$rcf"
  return "${rc:-1}"
}

# Changed files: merge base .. working tree, plus untracked, one per line.
if base=$(git merge-base HEAD origin/main 2>/dev/null); then
  changed=$( { git diff --name-only --no-renames "$base"; git ls-files --others --exclude-standard; } | sort -u)
else
  echo "prepush: no origin/main to compare with, treating everything as changed" >&2
  PREPUSH_FULL=1
  changed=""
fi
full="${PREPUSH_FULL:-0}"

# changed_any <extended regex>: a changed file matches.
changed_any() { [ "$full" = 1 ] || grep -Eq "$1" <<<"$changed"; }

# 1. fmt
begin "fmt"
scarb fmt --check --workspace
end

# 2. unit tests of the scripts (the ones that have a self-test; gas.py has none, CI runs it on the snforge logs)
begin "scripts self-tests"
python3 scripts/api_parity.py --self-test
python3 scripts/consumer_cost.py --self-test
python3 scripts/packages_table.py --self-test
end

# 3. compile the packages touched
begin "build/lint"
if [ "$full" = 1 ] || grep -Eq '^(Scarb\.toml|Scarb\.lock|\.tool-versions)$' <<<"$changed"; then
  note "root manifest, lock, toolchain or PREPUSH_FULL: whole workspace"
  locked_run scarb lint --workspace --deny-warnings
  locked_run scarb build --workspace
else
  # Only Cairo sources and manifests count: a README or a snapshot does not change what compiles.
  crates=$(sed -nE 's#^crates/([^/]+)/(Scarb\.toml|(src|tests|programs)/.*\.cairo)$#\1#p' <<<"$changed" | sort -u)
  if [ -z "$crates" ]; then
    note "no Cairo source or manifest changed: compile skipped"
  fi
  for c in $crates; do
    # A crate removed by the branch has no manifest left.
    [ -f "crates/$c/Scarb.toml" ] || continue
    note "crate $c"
    locked_run scarb lint -p "$c" --deny-warnings
    locked_run scarb build -p "$c"
  done
fi
end

# 4. generated artefacts, only when their inputs changed
begin "artefacts"
if changed_any '^(crates/[^/]+/src/.*\.cairo|docs/API_PARITY\.md|scripts/api_parity\.py)$'; then
  note "api-parity (a public source or the table changed): scripts/api_parity.py --check"
  python3 scripts/api_parity.py --check
fi

if changed_any '^gas/.*\.snap$'; then
  note "gas (a gas snapshot changed): needs the tests, not run here (minutes). Check the crates you touched with:"
  for d in $(sed -nE 's#^gas/([^/]+)/.*\.snap$#\1#p' <<<"$changed" | sort -u); do
    note "  python3 scripts/gas.py check --filter ${d}::"
  done
elif grep -Eq '^crates/[^/]+/(src|tests)/.*\.cairo$' <<<"$changed"; then
  note "gas: crate code changed but no snapshot did; CI's gas job compares the merged snforge logs"
fi

if changed_any '^(crates/(rapier_sink|rapier2d_classes|rapier_core|rapier_math|rapier_geometry2d|rapier_dynamics2d|rapier2d)/src/.*|crates/rapier_sink/programs/.*|crates/[^/]+/Scarb\.toml|Scarb\.toml|Scarb\.lock|\.tool-versions|scripts/bytecode_size\.py)$'; then
  note "WARNING gas/bytecode.size: class sources, a manifest or the toolchain changed, so CI's \`bytecode\` job will"
  note "  report new sizes. The file is generated in CI only (sierra_bytes depend on the build path): take it from the"
  note "  \`bytecode-snapshot\` artifact of the run (once TC1 adds it); never regenerate it here."
fi

if changed_any '^(crates/[^/]+/src/.*|crates/[^/]+/Scarb\.toml|Scarb\.toml|consumer_cost\.toml|scripts/(consumer_cost|packages_table)\.py)$'; then
  note "WARNING docs/PACKAGES.md: inputs of the package table changed; it is generated in CI only (\`consumer-cost\`"
  note "  artifact), update it from there if the job reports a change."
fi
end

echo "prepush: OK ($((SECONDS - start)) s, of which waiting for the build lock, upper bound: $lock_wait_total s)"

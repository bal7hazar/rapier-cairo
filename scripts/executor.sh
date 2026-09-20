#!/usr/bin/env bash
# Launch (or resume) one Executor sub-agent in an isolated worktree through a local CLI.
#
#   scripts/executor.sh <id> <runner> <brief.md>          launch
#   scripts/executor.sh resume <id> <runner> "<follow-up>" resume in the same worktree
#
#   id      work package id, lowercase (e.g. ga-broad-phase): branch feat/<id>, worktree
#           .claude/worktrees/exec-<id>, created from $EXECUTOR_BASE (default origin/main)
#   runner  claude:<sonnet|opus|fable>            e.g. claude:sonnet
#           codex:<model>[:<effort>]             e.g. codex:gpt-5.5:medium, codex:gpt-6-astra:xhigh
#   brief   Markdown brief in the mandatory format (docs/ORCHESTRATOR.md §"The brief")
#
# Environment:
#   EXECUTOR_LOG_DIR   where to write <id>.log (default: ./.executor-logs)
#   EXECUTOR_MAX_TURNS claude turn budget (default 300)
#   EXECUTOR_BASE      ref the new branch starts from (default: origin/main)
#
# The agent works with all permission prompts disabled inside its own worktree, opens its own PR
# and writes REPORT.md (not committed) at the worktree root: read that file and the log, not the
# transcript. Both CLIs use their own logins, distinct from the orchestrator's session.
set -euo pipefail

usage() { sed -n '2,20p' "$0"; exit 64; }

MODE=launch
if [ "${1:-}" = "resume" ]; then MODE=resume; shift; fi
[ $# -ge 3 ] || usage

ID="$1"; RUNNER="$2"; INPUT="$3"; shift 3
CLI="${RUNNER%%:*}"; REST="${RUNNER#*:}"; MODEL="${REST%%:*}"; EFFORT=""
[ "$REST" != "$MODEL" ] && EFFORT="${REST#*:}"

ROOT="$(git rev-parse --show-toplevel)"
COMMON="$(git -C "$ROOT" rev-parse --git-common-dir)"
REPO_ROOT="$(cd "$COMMON/.." && pwd)"
WORKTREE="$REPO_ROOT/.claude/worktrees/exec-$ID"
BRANCH="feat/$ID"
LOG_DIR="${EXECUTOR_LOG_DIR:-$ROOT/.executor-logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$ID.log"

if [ "$MODE" = launch ]; then
  [ -f "$INPUT" ] || { echo "brief not found: $INPUT" >&2; exit 66; }
  git -C "$ROOT" fetch -q origin
  [ -d "$WORKTREE" ] && { echo "worktree already exists: $WORKTREE" >&2; exit 73; }
  if git -C "$ROOT" show-ref --quiet "refs/heads/$BRANCH"; then
    git -C "$ROOT" worktree add -q "$WORKTREE" "$BRANCH"
  else
    git -C "$ROOT" worktree add -q -b "$BRANCH" "$WORKTREE" "${EXECUTOR_BASE:-origin/main}"
  fi
  cp "$INPUT" "$WORKTREE/BRIEF.md"
  PROMPT="$(cat "$ROOT/scripts/executor/system-prompt.md")

----

Your brief (also saved as BRIEF.md at the repository root). Your branch is $BRANCH, already
checked out in this worktree. Start now.

$(cat "$INPUT")"
else
  [ -d "$WORKTREE" ] || { echo "no worktree to resume: $WORKTREE" >&2; exit 66; }
  PROMPT="$INPUT"
fi

cd "$WORKTREE"
echo "executor $ID ($MODE): cli=$CLI model=$MODEL effort=${EFFORT:-default} branch=$BRANCH log=$LOG"
set +e
case "$CLI" in
  claude)
    ARGS=(-p --model "$MODEL" --max-turns "${EXECUTOR_MAX_TURNS:-300}" --name "exec-$ID" \
          --dangerously-skip-permissions --output-format stream-json --verbose)
    [ "$MODE" = resume ] && ARGS+=(--continue)
    env -u CLAUDECODE -u CLAUDE_CODE_SESSION_ID -u CLAUDE_CODE_CHILD_SESSION \
      claude "${ARGS[@]}" "$@" "$PROMPT" >> "$LOG"
    ;;
  codex)
    COMMON_ARGS=(-C "$WORKTREE" --dangerously-bypass-approvals-and-sandbox -o REPORT.md)
    [ -n "$EFFORT" ] && COMMON_ARGS+=(-c "model_reasoning_effort=$EFFORT")
    if [ "$MODE" = resume ]; then
      codex exec resume --last "${COMMON_ARGS[@]}" "$@" "$PROMPT" >> "$LOG" 2>&1
    else
      codex exec -m "$MODEL" "${COMMON_ARGS[@]}" "$@" "$PROMPT" >> "$LOG" 2>&1
    fi
    ;;
  *) echo "unknown runner: $RUNNER" >&2; exit 64 ;;
esac
STATUS=$?
set -e

if [ "$CLI" = claude ]; then
  python3 - "$LOG" <<'EOF'
import json, sys
result = None
for line in open(sys.argv[1]):
    try:
        msg = json.loads(line)
    except ValueError:
        continue
    if msg.get("type") == "result":
        result = msg
if result:
    usage = result.get("modelUsage", {})
    print(f"turns={result.get('num_turns')} cost_usd={result.get('total_cost_usd')} "
          + " ".join(f"{m}:out={u.get('outputTokens')}" for m, u in usage.items()))
EOF
fi
[ -f REPORT.md ] && echo "REPORT.md: $WORKTREE/REPORT.md" || echo "warning: no REPORT.md written"
echo "executor $ID finished with status $STATUS"
exit $STATUS

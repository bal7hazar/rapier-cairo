#!/usr/bin/env bash
# Launch one Executor sub-agent through the `claude` CLI in an isolated worktree.
#
#   scripts/executor.sh <id> <model> <brief.md> [extra claude args...]
#
#   id      work package id, lowercase (e.g. ga-broad-phase); branch feat/<id>, worktree
#           .claude/worktrees/exec-<id>, both created from origin/main
#   model   sonnet | opus | haiku | a full model id
#   brief   Markdown brief written by the orchestrator (see AGENTS.md §3)
#
# Environment:
#   EXECUTOR_LOG_DIR   where to write <id>.jsonl and <id>.result (default: ./.executor-logs)
#   EXECUTOR_MAX_TURNS turn budget (default 200)
#   EXECUTOR_EFFORT    effort level passed to the CLI (default: high)
#
# The CLI runs with its own login (a separate account from the orchestrator's session), with the
# permission prompts replaced by an explicit allow-list: file edits, and Bash restricted to the
# toolchain, git and python. Everything else is denied. Nested-session variables are unset so the
# CLI does not believe it runs inside another Claude Code session.
set -euo pipefail

if [ $# -lt 3 ]; then
  sed -n '2,20p' "$0"
  exit 64
fi

ID="$1"
MODEL="$2"
BRIEF="$3"
shift 3

ROOT="$(git rev-parse --show-toplevel)"
COMMON="$(git -C "$ROOT" rev-parse --git-common-dir)"
REPO_ROOT="$(cd "$COMMON/.." && pwd)"
WORKTREE="$REPO_ROOT/.claude/worktrees/exec-$ID"
BRANCH="feat/$ID"
LOG_DIR="${EXECUTOR_LOG_DIR:-$ROOT/.executor-logs}"
mkdir -p "$LOG_DIR"

[ -f "$BRIEF" ] || { echo "brief not found: $BRIEF" >&2; exit 66; }

git -C "$ROOT" fetch -q origin
if [ -d "$WORKTREE" ]; then
  echo "worktree already exists: $WORKTREE" >&2
  exit 73
fi
if git -C "$ROOT" show-ref --quiet "refs/heads/$BRANCH"; then
  git -C "$ROOT" worktree add -q "$WORKTREE" "$BRANCH"
else
  git -C "$ROOT" worktree add -q -b "$BRANCH" "$WORKTREE" origin/main
fi
cp "$BRIEF" "$WORKTREE/.executor-brief.md"
cp "$ROOT/scripts/executor/system-prompt.md" "$WORKTREE/.executor-system.md"

PROMPT="Your work package brief is in .executor-brief.md at the root of this repository (also \
reproduced below). Your branch is $BRANCH, already checked out in this worktree. Start now.

$(cat "$BRIEF")"

cd "$WORKTREE"
echo "executor $ID: model=$MODEL branch=$BRANCH worktree=$WORKTREE log=$LOG_DIR/$ID.jsonl"
set +e
env -u CLAUDECODE -u CLAUDE_CODE_SESSION_ID -u CLAUDE_CODE_CHILD_SESSION \
  claude -p \
  --model "$MODEL" \
  --effort "${EXECUTOR_EFFORT:-high}" \
  --max-turns "${EXECUTOR_MAX_TURNS:-200}" \
  --name "exec-$ID" \
  --append-system-prompt-file .executor-system.md \
  --permission-mode acceptEdits \
  --allowedTools "Read" "Edit" "Write" "Glob" "Grep" "MultiEdit" \
    "Bash(scarb:*)" "Bash(snforge:*)" "Bash(git:*)" "Bash(python3:*)" "Bash(cargo:*)" \
    "Bash(ls:*)" "Bash(cat:*)" "Bash(wc:*)" "Bash(head:*)" "Bash(tail:*)" "Bash(grep:*)" \
    "Bash(find:*)" "Bash(diff:*)" "Bash(mkdir:*)" "Bash(sed:*)" "Bash(awk:*)" "Bash(sort:*)" \
  --output-format stream-json --verbose \
  "$@" \
  "$PROMPT" > "$LOG_DIR/$ID.jsonl"
STATUS=$?
set -e

python3 - "$LOG_DIR/$ID.jsonl" "$LOG_DIR/$ID.result" <<'EOF'
import json, sys
log, out = sys.argv[1], sys.argv[2]
result = None
for line in open(log):
    try:
        msg = json.loads(line)
    except ValueError:
        continue
    if msg.get("type") == "result":
        result = msg
if result is None:
    open(out, "w").write("no result message in log\n")
    sys.exit(0)
usage = result.get("modelUsage", {})
summary = [
    f"is_error: {result.get('is_error')}",
    f"num_turns: {result.get('num_turns')}",
    f"duration_ms: {result.get('duration_ms')}",
    f"total_cost_usd: {result.get('total_cost_usd')}",
    "models: " + ", ".join(f"{m} in={u.get('inputTokens')} out={u.get('outputTokens')}" for m, u in usage.items()),
    "",
    result.get("result") or "",
]
open(out, "w").write("\n".join(summary) + "\n")
print("\n".join(summary))
EOF

echo "executor $ID finished with status $STATUS"
exit $STATUS

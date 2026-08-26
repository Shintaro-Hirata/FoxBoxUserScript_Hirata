#!/usr/bin/env bash
# ============================================================================
# Verify all User Scripts: strict type-check + synthetic-data behavior tests.
#
# Usage:  bash test/run.sh
#
# Requires `tsc` and `node` on PATH (both are present in Claude Code
# environments). Equivalent to `npm run verify` but without needing an
# `npm install` first.
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== [1/3] strict type-check (tsc -p tsconfig.json) =="
tsc -p tsconfig.json

echo "== [2/3] build tests (tsc -p test/tsconfig.build.json) =="
tsc -p test/tsconfig.build.json

echo "== [3/3] behavior tests (node test/behavior.test.cjs) =="
node test/behavior.test.cjs

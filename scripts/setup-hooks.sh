#!/usr/bin/env bash
# Install git hooks for tracematch. Run this once after cloning.
#
# Copying into .git/hooks would leave the guard behind on the next clone, and
# the thing it guards is personal GPS data. Pointing core.hooksPath at a tracked
# directory means the hook travels with the repository.

set -e

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

git config core.hooksPath .githooks
chmod +x .githooks/* 2>/dev/null || true

echo "core.hooksPath -> .githooks"
echo
echo "pre-commit now refuses to stage GPS traces and checks formatting."
echo "It is still per-clone local config, so run this again on any new clone."

#!/usr/bin/env bash
# Install git hooks for tracematch

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
HOOKS_DIR="$REPO_ROOT/.git/hooks"

echo "Installing git hooks..."

# Install pre-commit hook
cp "$SCRIPT_DIR/pre-commit" "$HOOKS_DIR/pre-commit"
chmod +x "$HOOKS_DIR/pre-commit"

echo "✓ Installed pre-commit hook"
echo ""
echo "Hooks installed. They will run automatically on git commit."
echo "To skip hooks temporarily: git commit --no-verify"

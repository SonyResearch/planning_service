#!/bin/bash

# Script which evaluates the current refs of any submodules against the `main` remote.
# Must be behind or up-to-date in order to pass (i.e., no commits not tracked by the submodule).
set -euo pipefail

FAILED=false
echo "Checking submodules..."
for directory in external/*/; do
  pushd $directory >/dev/null
  echo "Checking submodule: $directory"
  if [[ ! -f .git ]]; then
    echo "Not a submodule (no .git file found), skipping: $directory"
    popd >/dev/null
    continue
  fi
  SUBMODULE=$(basename $(git rev-parse --show-toplevel))
  echo "Fetching latest changes from origin/main for submodule: $SUBMODULE"
  git fetch -q origin main
  COMMITS_AHEAD=$(git rev-list --count origin/main..HEAD)
  if [[ COMMITS_AHEAD -gt 0 ]]; then
    HEAD_REF=$(git rev-parse --short HEAD)
    MAIN_REF=$(git rev-parse --short origin/main)
    echo "FAIL: \`${SUBMODULE}:HEAD\` (${HEAD_REF}) is ahead of \`${SUBMODULE}:origin/main\` (${MAIN_REF}) by ${COMMITS_AHEAD} commit(s)!"
    FAILED=true
  fi
  popd >/dev/null
done

if [[ $FAILED == true ]]; then
  echo ""
  echo "New changes to submodules must be merged in their respective repositories before being updated in this project."
  echo "Please stash or discard your local changes in order to proceed."
  exit 1
fi

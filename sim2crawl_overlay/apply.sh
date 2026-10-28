#!/usr/bin/env bash
# Applies the crawl changes to YOUR FORK of lez666-g1-sim2crawl.
# Run from the root of that fork:   bash /path/to/apply.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
[ -f sim2sim_mj/run_sim2sim_keyboard.py ] || { echo "Run this from the root of lez666-g1-sim2crawl"; exit 1; }
git checkout -b crawl-transition 2>/dev/null || git checkout crawl-transition
git apply --check "$here/g1_crawl_changes.patch" && git apply "$here/g1_crawl_changes.patch"
python3 -m unittest discover -s sim2sim_mj -p 'test_crawl_sequencer.py' -v
echo "Applied. Run:  python sim2sim_mj/run_sim2sim_keyboard.py"

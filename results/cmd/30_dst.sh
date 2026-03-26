#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

RESULTS_DIR="$ROOT_DIR/results/30_dst"

mkdir -p "$RESULTS_DIR"

cd "$BUILD_DIR"

./dynamic_environment generate-points 1000 30

cp "$ROOT_DIR/data/points.txt" "$RESULTS_DIR/points.txt"
./dynamic_environment test static > "$RESULTS_DIR/static.txt"

./dynamic_environment generate 5000 1
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_1.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_1.txt"

./dynamic_environment generate 5000 3
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_3.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_3.txt"

./dynamic_environment generate 5000 5
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_5.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_5.txt"

./dynamic_environment generate 5000 7
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_7.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_7.txt"

./dynamic_environment generate 5000 10
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_10.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_10.txt"

./dynamic_environment generate 5000 13
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_13.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_13.txt"

./dynamic_environment generate 5000 15
cp "$ROOT_DIR/data/changes.txt" "$RESULTS_DIR/changes_5000_15.txt"
./dynamic_environment test dynamic > "$RESULTS_DIR/dynamic_5000_15.txt"

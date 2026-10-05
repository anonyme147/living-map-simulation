#!/usr/bin/env bash
set -u
cd "$(dirname "$0")/.."
exec python3 tests/run_tests.py "$@"

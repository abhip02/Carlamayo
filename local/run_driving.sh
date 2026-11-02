#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$CARLAMAYO_ROOT"
exec .venv/bin/python -m local.run_driving "$@"

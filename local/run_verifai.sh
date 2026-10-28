#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$CARLAMAYO_ROOT"
sampler_python="${CARLAMAYO_ROOT}/.runtime/verifai-venv/bin/python"
if [[ ! -x "$sampler_python" ]]; then
  echo "Install the sampler first: bash local/setup_verifai.sh" >&2
  exit 1
fi
export PYGAME_HIDE_SUPPORT_PROMPT=1
exec "$sampler_python" -m local.run_verifai "$@"

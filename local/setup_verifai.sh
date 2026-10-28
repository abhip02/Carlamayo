#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$CARLAMAYO_ROOT"
sampler_env="${CARLAMAYO_ROOT}/.runtime/verifai-venv"
if [[ ! -x "$sampler_env/bin/python" ]]; then
  uv venv --python 3.12 "$sampler_env"
fi
uv pip install --python "$sampler_env/bin/python" -r local/requirements-verifai.txt

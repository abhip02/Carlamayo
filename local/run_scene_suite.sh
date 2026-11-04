#!/usr/bin/env bash
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
: "${CARLAMAYO_MODEL_GPUS:?Set one available BF16-capable model GPU UUID}"
: "${CARLAMAYO_RENDER_GPU:?Set the rendering GPU UUID}"
export CARLAMAYO_PROJECTION_CONTAINER="${CARLAMAYO_PROJECTION_CONTAINER:-apomalap-carlamayo-ten-scenes}"
cd "$repo"
source local/env.sh
exec .venv/bin/python -u -m local.experiments.run_projection_suite \
  --manifest local/experiments/presets/ten-driving-scenes.json \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 6 \
  --speed-cap-mps 4.5 --steering-gain 1.18 --cache-age-s 3 \
  --cases 1 2 3 4 5 6 7 8 9 10 "$@"

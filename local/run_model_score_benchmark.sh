#!/usr/bin/env bash
# Reproduce the paired raw-model experiment with a fresh output directory.
# Dependencies are isolated from the pinned CARLA environment:
# uv pip install --python .venv/bin/python --target .runtime/quant-benchmark-deps \
#   --no-deps bitsandbytes==0.48.2
# Usage: CARLAMAYO_MODEL_GPUS=GPU-UUID bash local/run_model_score_benchmark.sh OUTPUT
set -euo pipefail
if [[ $# != 1 ]]; then
    echo 'Usage: CARLAMAYO_MODEL_GPUS=GPU-UUID bash local/run_model_score_benchmark.sh OUTPUT' >&2
    exit 2
fi
: "${CARLAMAYO_MODEL_GPUS:?Select one idle GPU by UUID}"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source local/env.sh
export PYTHONPATH="${CARLAMAYO_ROOT}/.runtime/quant-benchmark-deps:${CARLAMAYO_ROOT}"
benchmark_output="$1"
.venv/bin/python -c 'import bitsandbytes; assert bitsandbytes.__version__ == "0.48.2"'
.venv/bin/python -m local.experiments.model_score_benchmark prepare --output "$benchmark_output"
.venv/bin/python -m local.experiments.model_score_benchmark run --precision bf16 \
    --output "$benchmark_output" > "$benchmark_output/bf16-console.log" 2>&1
.venv/bin/python -m local.experiments.model_score_benchmark run --precision nf4 \
    --export-checkpoint --output "$benchmark_output" > "$benchmark_output/nf4-console.log" 2>&1
.venv/bin/python -m local.experiments.model_score_benchmark summarize --output "$benchmark_output"
.venv/bin/python -m local.experiments.verify_model_scores "$benchmark_output" --reload-checkpoint \
    > "$benchmark_output/verification-console.log" 2>&1

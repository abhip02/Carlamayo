#!/usr/bin/env bash
# Run numerical activation stress tests after the paired BF16 baseline completes.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
: "${CARLAMAYO_MODEL_GPUS:?Set CARLAMAYO_MODEL_GPUS to one available A5000 UUID}"
if [[ "$CARLAMAYO_MODEL_GPUS" == *,* ]]; then
    echo "Select exactly one model GPU for NF4 variants" >&2
    exit 2
fi
source local/env.sh
export PYTHONPATH="$CARLAMAYO_ROOT/.runtime/quant-benchmark-deps:$CARLAMAYO_ROOT"
benchmark_output="$(realpath -- "${1:?Pass the existing maneuver benchmark output directory}")"
test -f "$benchmark_output/bf16/complete.json"
for benchmark_variant in nf4_a8 nf4_a4; do
    .venv/bin/python -m local.experiments.resource_lease \
        --gpus "$CARLAMAYO_MODEL_GPUS" --minimum-free-gib 10 \
        -- .venv/bin/python -c \
        'import torch,runpy; torch.cuda.set_per_process_memory_fraction(.41); torch.cuda.set_device(0); runpy.run_module("local.experiments.quant_explore", run_name="__main__")' \
        run --output "$benchmark_output" --variant "$benchmark_variant" \
        > "$benchmark_output/$benchmark_variant.log" 2>&1
done
.venv/bin/python -m local.experiments.analyze_quant_explore "$benchmark_output" \
    > "$benchmark_output/analysis.log" 2>&1
.venv/bin/python -m local.experiments.verify_quant_explore "$benchmark_output" \
    > "$benchmark_output/verification.log" 2>&1

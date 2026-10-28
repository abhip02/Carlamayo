#!/usr/bin/env bash
# Source this file; all writable caches stay inside this checkout.
CARLAMAYO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export CARLAMAYO_ROOT
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
unset TRANSFORMERS_CACHE PYTORCH_TRANSFORMERS_CACHE PYTORCH_PRETRAINED_BERT_CACHE
export HF_HOME="${CARLAMAYO_ROOT}/../cache/huggingface"
export HF_HUB_CACHE="${HF_HOME}/hub"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export XDG_CACHE_HOME="${CARLAMAYO_ROOT}/.runtime/cache"
export MPLCONFIGDIR="${CARLAMAYO_ROOT}/.runtime/matplotlib"
export TMPDIR="${CARLAMAYO_ROOT}/.runtime/tmp"
export UV_CACHE_DIR="${CARLAMAYO_ROOT}/.runtime/uv-cache"
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export TOKENIZERS_PARALLELISM=false
# Local cuda:0 = host GPU 2; local cuda:1 = host GPU 1.
export CUDA_VISIBLE_DEVICES="${CARLAMAYO_MODEL_GPUS:-GPU-216b01e6-03fa-563d-0ff0-739ef6f558f8,GPU-46394453-71bb-15b9-1612-860b5714f750}"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME" "$MPLCONFIGDIR"

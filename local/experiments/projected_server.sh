#!/usr/bin/env bash
# Separate owned CARLA container so rendering can use an idle non-BF16 GPU.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
name=apomalap-carlamayo-projection
render_gpu="${CARLAMAYO_RENDER_GPU:?Set CARLAMAYO_RENDER_GPU explicitly}"
image=carlasim/carla@sha256:aaf1df22702780ece072069e23d03c4879b002ae028c79744b09c4c7ddbae953
if docker container inspect "$name" >/dev/null 2>&1; then
  owner="$(docker inspect --format '{{index .Config.Labels "apomalap.carlamayo.root"}}' "$name")"
  assigned="$(docker inspect --format '{{index .Config.Labels "apomalap.carlamayo.render_gpu"}}' "$name")"
  [[ "$owner" == "$repo" && "$assigned" == "$render_gpu" ]] || { echo 'Projection container ownership/GPU mismatch' >&2; exit 1; }
fi
case "${1:-start}" in
  start)
    if docker container inspect "$name" >/dev/null 2>&1; then
      docker start "$name"
    else
      docker run -d --name "$name" --label "apomalap.carlamayo.root=${repo}" \
        --label "apomalap.carlamayo.render_gpu=${render_gpu}" \
        --device "nvidia.com/gpu=${render_gpu}" --network=host --shm-size=2g \
        -e NVIDIA_VISIBLE_DEVICES=void "$image" \
        bash CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
    fi ;;
  stop) docker stop --timeout 15 "$name" ;;
  *) echo 'Usage: projected_server.sh [start|stop]' >&2; exit 2 ;;
esac

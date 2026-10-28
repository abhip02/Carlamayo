#!/usr/bin/env bash
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
name=apomalap-carlamayo-0916
label=apomalap.carlamayo.root
image=carlasim/carla@sha256:aaf1df22702780ece072069e23d03c4879b002ae028c79744b09c4c7ddbae953
action="${1:-start}"
if docker container inspect "$name" >/dev/null 2>&1; then
  owner="$(docker inspect --format '{{index .Config.Labels "apomalap.carlamayo.root"}}' "$name")"
  [[ "$owner" == "$repo" ]] || { echo "Container name belongs to another setup" >&2; exit 1; }
fi
case "$action" in
  start)
    if docker container inspect "$name" >/dev/null 2>&1; then
      docker start "$name"
    else
      docker run -d --name "$name" --label "${label}=${repo}" \
        --device "nvidia.com/gpu=${CARLAMAYO_RENDER_GPU:-GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b}" \
        --network=host --shm-size=2g \
        -e NVIDIA_VISIBLE_DEVICES=void \
        "$image" bash CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
    fi
    ;;
  stop) docker stop --timeout 15 "$name" ;;
  remove) docker rm -f "$name" ;;
  logs) docker logs --tail 60 "$name" ;;
  *) echo 'Usage: bash local/server.sh [start|stop|remove|logs]' >&2; exit 2 ;;
esac

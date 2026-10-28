#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
base_env="${CARLAMAYO_ROOT}/../.venv-alpamayo"
if [[ ! -x "${CARLAMAYO_ROOT}/.venv/bin/python" ]]; then
  uv venv --python "${base_env}/bin/python" "${CARLAMAYO_ROOT}/.venv"
fi
# Reuse the working ML dependencies without modifying or duplicating them.
# addsitedir also resolves the base environment's editable Alpamayo install.
"${CARLAMAYO_ROOT}/.venv/bin/python" - "$base_env" <<'PY'
from pathlib import Path
import sys, sysconfig
base = Path(sys.argv[1]).resolve() / 'lib/python3.12/site-packages'
dest = Path(sysconfig.get_path('purelib')) / 'carlamayo_base.pth'
dest.write_text(f'import site; site.addsitedir({str(base)!r})\n')
PY
uv pip install --python "${CARLAMAYO_ROOT}/.venv/bin/python" --no-deps \
  carla==0.9.16 opencv-python-headless==4.13.0.92
"${CARLAMAYO_ROOT}/.venv/bin/python" - <<'PY'
import carla, cv2, torch, alpamayo1_5
print('CARLA client, OpenCV, PyTorch and Alpamayo imports OK')
print('torch:', torch.__version__, 'opencv:', cv2.__version__)
print('Alpamayo source:', alpamayo1_5.__path__)
PY

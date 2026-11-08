"""Cooperative GPU/CARLA leases for experiments sharing this workspace."""
from contextlib import contextmanager
import csv
import fcntl
import json
import os
from pathlib import Path
import re
import socket
import subprocess


def validate_gpus(model_gpu, render_gpu):
    pattern = r'GPU-[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}'
    if not all(re.fullmatch(pattern, value or '') for value in (model_gpu, render_gpu)):
        raise RuntimeError('Set one full GPU UUID in each of CARLAMAYO_MODEL_GPUS and CARLAMAYO_RENDER_GPU')
    if model_gpu == render_gpu:
        raise RuntimeError('Latency pilot requires separate model and renderer GPUs')


@contextmanager
def acquire(directory, model_gpu, render_gpu):
    validate_gpus(model_gpu, render_gpu)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    handles = []
    try:
        for name in sorted([f'gpu-{model_gpu}.lock', f'gpu-{render_gpu}.lock', 'carla-port-2000.lock']):
            handle = (directory / name).open('a+')
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                handle.close()
                raise RuntimeError(f'Resource already leased: {name}') from None
            handles.append(handle)
            handle.seek(0)
            handle.truncate()
            handle.write(json.dumps({'pid': os.getpid(), 'worktree': str(Path.cwd())}) + '\n')
            handle.flush()
        yield
    finally:
        for handle in reversed(handles):
            handle.seek(0)
            handle.truncate()
            handle.flush()
            handle.close()  # Closing releases the flock, including on exceptions.


def gpu_snapshot(gpu):
    output = subprocess.check_output(['nvidia-smi', '-i', gpu,
        '--query-gpu=uuid,name,memory.used', '--format=csv,noheader,nounits'], text=True)
    rows = list(csv.reader(output.splitlines(), skipinitialspace=True))
    if len(rows) != 1 or rows[0][0] != gpu:
        raise RuntimeError(f'Unexpected GPU query result for {gpu}')
    return dict(uuid=gpu, name=rows[0][1], memory_used_mib=int(rows[0][2]))


def require_idle(model_gpu, render_gpu, ports=(2000, 2001, 2002)):
    snapshots = [gpu_snapshot(gpu) for gpu in (model_gpu, render_gpu)]
    if 'RTX A5000' not in snapshots[0]['name']:
        raise RuntimeError('Frozen latency pilot requires an A5000 model GPU')
    for snapshot in snapshots:
        if snapshot['memory_used_mib'] > 256:
            raise RuntimeError(f"GPU is occupied: {snapshot['uuid']} ({snapshot['memory_used_mib']} MiB); no job launched")
    for port in ports:
        with socket.socket() as connection:
            connection.settimeout(.3)
            if connection.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError(f'CARLA port {port} is occupied; no job launched')
    return snapshots

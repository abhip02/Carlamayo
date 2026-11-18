"""Nonblocking leases coordinating our experiments, not external GPU jobs."""
import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

LOCK_ROOT = Path('/scratch/apomalap/thesis/.carlamayo-resource-locks')


@contextmanager
def lease(gpus, port=None, lock_root=LOCK_ROOT):
    names = ['gpu-' + gpu for gpu in gpus]
    if any(not gpu.startswith('GPU-') or '/' in gpu for gpu in gpus):
        raise ValueError('Use full GPU UUIDs')
    if port is not None:
        names.append(f'carla-port-{port}')
    handles = []
    lock_root.mkdir(parents=True, exist_ok=True)
    try:
        for name in sorted(set(names)):
            stream = (lock_root / (name + '.lock')).open('a+')
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                stream.close()
                raise RuntimeError(f'Resource already leased: {name}') from None
            handles.append(stream)
            stream.seek(0)
            stream.truncate()
            json.dump(dict(pid=os.getpid(), started_unix=time.time(), resource=name), stream)
            stream.flush()
        yield
    finally:
        for stream in reversed(handles):
            fcntl.flock(stream, fcntl.LOCK_UN)
            stream.close()


def check_memory(gpus, minimum_free_gib):
    rows = subprocess.check_output(['nvidia-smi', '--query-gpu=uuid,memory.free', '--format=csv,noheader,nounits'], text=True)
    free = {parts[0].strip(): float(parts[1]) / 1024 for row in rows.splitlines() if (parts := row.split(','))}
    for gpu, minimum in zip(gpus, minimum_free_gib, strict=True):
        if free[gpu] < minimum:
            raise RuntimeError(f'Insufficient free memory on {gpu}: {free[gpu]:.2f} GiB < {minimum:.2f} GiB')
    return {gpu: free[gpu] for gpu in gpus}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpus', nargs='+', required=True)
    parser.add_argument('--minimum-free-gib', nargs='+', type=float, required=True)
    parser.add_argument('--port', type=int)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('A command is required')
    with lease(args.gpus, args.port):
        print(json.dumps(dict(leased=args.gpus, port=args.port,
                              free_gib=check_memory(args.gpus, args.minimum_free_gib))), flush=True)
        raise SystemExit(subprocess.call(command))


if __name__ == '__main__':
    main()

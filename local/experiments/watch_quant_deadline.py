"""Bound this workspace's existing A4 run; never signal an unrelated process."""
import argparse
import json
import os
from pathlib import Path
import signal
import time


def identity(pid, output):
    folder = Path('/proc') / str(pid)
    try:
        if folder.stat().st_uid != os.getuid():
            raise ValueError('Process owner does not match')
        args = (folder / 'cmdline').read_bytes().decode().strip('\0').split('\0')
        if not args[0]:
            return None  # exited process awaiting its parent's wait()
        if len(args) < 8 or args[1] != '-c' or 'local.experiments.quant_explore' not in args[2]:
            raise ValueError('Not our quantization runner')
        if args[-2:] != ['--variant', 'nf4_a4']:
            raise ValueError('Not the A4 stress test')
        if Path(args[args.index('--output')+1]).resolve() != output.resolve():
            raise ValueError('Benchmark output differs')
        # Everything after the parenthesized process name begins with stat field3.
        stat = (folder / 'stat').read_text().rsplit(')', 1)[1].split()
        return int(stat[19])
    except FileNotFoundError:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=2100.)
    args = parser.parse_args()
    start = identity(args.pid, args.output)
    if start is None:
        return
    age = float(Path('/proc/uptime').read_text().split()[0]) - start/os.sysconf('SC_CLK_TCK')
    deadline = time.monotonic() + args.seconds - age
    report = dict(pid=args.pid, process_start_ticks=start, limit_seconds=args.seconds,
                  elapsed_at_watch_start_seconds=age, status='watching')
    path = args.output / 'a4-deadline.json'
    def save():
        path.write_text(json.dumps(report, indent=2)+'\n')
    save()
    print(json.dumps(report), flush=True)
    while identity(args.pid, args.output) == start and time.monotonic() < deadline:
        time.sleep(min(5., max(0., deadline-time.monotonic())))
    if identity(args.pid, args.output) != start:
        report['status'] = 'exited_before_deadline'
    else:
        report['status'] = 'deadline_reached'
        report['signals'] = []
        for sig, grace in ((signal.SIGINT, 10.), (signal.SIGTERM, 5.), (signal.SIGKILL, 0.)):
            if identity(args.pid, args.output) != start:
                break
            os.kill(args.pid, sig)
            report['signals'].append(sig.name)
            save()
            if grace:
                time.sleep(grace)
    save()
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()

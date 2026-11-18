"""Run a VerifAI server and one persistent CarlaMayo client, scoring collisions."""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import random
import socket
import subprocess
import sys
import time

import numpy as np
import yaml
from dotmap import DotMap
with redirect_stdout(sys.stderr):
    from verifai.monitor import specification_monitor
    from verifai.samplers import FeatureSampler
    from verifai.server import Server

from local.run_verifai import ROOT, feature_space, rollout_spec, sampling_settings
from local.rollout_trace import TRACE_SCHEMA_VERSION, validate_result
from local.verifai_client import send_json


def collision_free(result):
    """Basic specification: no recorded collision during the evaluated episode."""
    summary = result["summary"]
    if not isinstance(summary, dict) or summary.get("error"):
        raise RuntimeError(f"Simulation failed: {summary}")
    termination = summary.get("termination")
    if termination not in ("collision", "duration_reached"):
        raise RuntimeError(f"Simulation did not complete: {termination!r}")
    return -1.0 if termination == "collision" or summary.get("collision") else 1.0


def make_sampler(ranges, method):
    constructors = {
        "random": FeatureSampler.randomSamplerFor,
        "halton": FeatureSampler.haltonSamplerFor,
        "ce": FeatureSampler.crossEntropySamplerFor,
    }
    return constructors[method](feature_space(ranges))


class RolloutServer(Server):
    """Real VerifAI Server with JSON transport and bounded local client waits.

    JSON keeps VerifAI's dependencies out of the existing GPU environment.
    Server.run_server() still owns sampling, monitor evaluation and feedback.
    Monitors receive {summary, trace}; trace validation always precedes scoring.
    """

    def __init__(self, sampler, ranges, count, port=8888, timeout=1200, on_sample=None,
                 sampler_type="random", spec_for_sample=None, monitor=None):
        super().__init__(DotMap(sampler=sampler, sample_space=None, sampler_type=sampler_type),
                         monitor if monitor is not None else specification_monitor(collision_free),
                         DotMap(port=port))
        self.port = self.socket.getsockname()[1]
        self.socket.settimeout(min(0.25, timeout))
        self.ranges, self.count, self.timeout = ranges, count, timeout
        self.on_sample = on_sample
        self.spec_for_sample = spec_for_sample or (lambda sample: rollout_spec(sample, ranges))
        self.worker = None
        self.index = 0
        self.current_spec = None
        self.last_result = None

    def listen(self):
        deadline = time.monotonic() + self.timeout
        while True:
            if self.worker is not None and self.worker.poll() is not None:
                raise RuntimeError(f"Simulation client exited with code {self.worker.returncode}")
            try:
                self.client_socket, _ = self.socket.accept()
                self.client_socket.settimeout(self.timeout)
                return
            except socket.timeout:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out waiting for the CarlaMayo client")

    def send(self, sample):
        self.current_spec = self.spec_for_sample(sample)
        if self.on_sample is not None:
            self.on_sample(self.current_spec)
        send_json(self.client_socket, {
            "index": self.index, "last": self.index + 1 == self.count,
            "spec": self.current_spec,
        })

    def decode(self, data):
        if not data:
            raise ConnectionError("Simulation client disconnected without a result")
        response = json.loads(data)
        if response.get("index") != self.index:
            raise ValueError("Received a result for the wrong episode")
        return {"summary": response["summary"], "trace": response.get("trace")}

    def evaluate_sample(self, sample):
        self.last_result = None
        self.listen()
        try:
            self.send(sample)
            self.last_result = self.receive()
        finally:
            self.close_connection()
        validate_result(self.last_result)
        return self.monitor.evaluate(self.last_result)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "local/configs/verifai.yaml")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--sampler", choices=("random", "halton", "ce"), default=None,
                        help="Feature-space sampler (default: random); exclusive with --scenic")
    source.add_argument("--scenic", type=Path, help="Scenic file defining the four initial-state params")
    parser.add_argument("--scenic-max-iterations", type=int, default=2000,
                        help="Maximum rejection-sampling attempts per Scenic scene (default: 2000)")
    parser.add_argument("--sample-only", action="store_true",
                        help="Print replayable YAML without starting the client or opening sockets")
    parser.add_argument("--limit", type=int, help="Cap sampling.count")
    parser.add_argument("--duration", type=float, help="Simulation seconds per rollout")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verifai-port", type=int, default=8888, help="Local coordinator port; 0 selects a free port")
    parser.add_argument("--timeout", type=float, default=1200, help="Client connection/socket inactivity timeout in seconds")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port")
    parser.add_argument("--checkpoint")
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text())
    try:
        count, seed, ranges = sampling_settings(config, args.limit, require_ranges=args.scenic is None)
    except ValueError as exc:
        parser.error(str(exc))
    config.setdefault("seed", 42)
    if args.duration is not None:
        config["duration_s"] = args.duration
    if not math.isfinite(config["duration_s"]) or config["duration_s"] < 0.1:
        parser.error("duration must be finite and at least 0.1 seconds")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    if not 0 <= args.verifai_port <= 65535:
        parser.error("--verifai-port must be between 0 and 65535")
    if args.scenic_max_iterations < 1:
        parser.error("--scenic-max-iterations must be positive")
    if args.sample_only and args.output is not None:
        parser.error("--sample-only writes YAML to stdout; use shell redirection instead of --output")
    random.seed(seed)
    np.random.seed(seed)
    sampler_type = "scenic" if args.scenic else (args.sampler or "random")
    spec_for_sample = None
    scenic_info = None
    try:
        # Scenic compilation may print diagnostics; keep preview stdout valid YAML.
        with redirect_stdout(sys.stderr):
            if args.scenic:
                from local.scenic_sampling import IntersectionScenicSampler

                scenario_path = args.scenic.resolve()
                scenario_source = scenario_path.read_bytes()
                sampler = IntersectionScenicSampler.fromScenario(
                    str(scenario_path), maxIterations=args.scenic_max_iterations)
                spec_for_sample = sampler.rollout_spec
                scenic_info = {
                    "path": str(scenario_path), "version": version("scenic"),
                    "sha256": hashlib.sha256(scenario_source).hexdigest(),
                    "max_iterations": args.scenic_max_iterations,
                }
                # Ranges are supplied by Scenic; don't record unused YAML bounds.
                config = {**config, "sampling": {key: value for key, value in config["sampling"].items()
                                                 if key != "ranges"}}
            else:
                sampler = make_sampler(ranges, sampler_type)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    resolved = {**config, "rollouts": [], "expanded_rollouts": [], "sampling_info": {
        "library": "verifai", "version": version("verifai"), "sampler": sampler_type,
        "mode": "sample_only" if args.sample_only else "server_client",
        "seed": seed, "sample_count": count,
        "specification": "no_recorded_collision", "passing_score": 1, "failing_score": -1,
        "trace_schema_version": TRACE_SCHEMA_VERSION,
    }}
    if scenic_info is not None:
        resolved["sampling_info"]["scenic"] = scenic_info
    if args.sample_only:
        convert = spec_for_sample or (lambda sample: rollout_spec(sample, ranges))
        with redirect_stdout(sys.stderr):
            specs = [convert(sampler.nextSample()) for _ in range(count)]
        resolved.update(rollouts=specs, expanded_rollouts=specs)
        print(yaml.safe_dump(resolved, sort_keys=False), end="")
        return 0

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "outputs" / f"verifai-server-{stamp}").resolve()
    output.mkdir(parents=True, exist_ok=False)
    if scenic_info is not None:
        (output / "scenario.scenic").write_bytes(scenario_source)
    write_json(output / "config.json", resolved)

    def save_sample(spec):
        resolved["rollouts"].append(spec)
        resolved["expanded_rollouts"].append(spec)
        write_json(output / "config.json", resolved)

    server = None
    worker = None
    scores = []
    status = {"status": "error", "requested_rollouts": count, "specification": "no_recorded_collision"}
    try:
        server = RolloutServer(sampler, ranges, count, args.verifai_port, args.timeout,
                               save_sample, sampler_type, spec_for_sample)
        print(f"VerifAI server listening on 127.0.0.1:{server.port}; "
              f"sampler={sampler_type}, rollouts={count}", flush=True)
        command = [str(ROOT / ".venv/bin/python"), "-m", "local.verifai_client",
                   "--config", str(output / "config.json"), "--output", str(output),
                   "--verifai-port", str(server.port), "--timeout", str(args.timeout),
                   "--host", args.host, "--port", str(args.port)]
        if args.checkpoint:
            command.extend(("--checkpoint", args.checkpoint))
        worker = subprocess.Popen(command, cwd=ROOT)
        server.worker = worker
        with (output / "verifai-results.jsonl").open("w", buffering=1) as journal:
            for index in range(count):
                server.index = index
                _, score, timings = server.run_server()
                result = {"index": index, "requested": server.current_spec,
                          "score": score, "passed": score > 0, **server.last_result,
                          "sample_seconds": timings.sample_time,
                          "evaluation_seconds": timings.simulate_time}
                journal.write(json.dumps(result, allow_nan=False) + "\n")
                scores.append(score)
                print(f"Episode {index}: collision-free score={score:+.0f}", flush=True)
        if worker.wait(timeout=30) != 0:
            raise RuntimeError(f"Simulation client exited with code {worker.returncode}")
        status["status"] = "completed"
    except BaseException as exc:
        status["error"] = f"{type(exc).__name__}: {exc}"
        if server is not None:
            status.update(failed_index=server.index, failed_result=server.last_result)
        raise
    finally:
        if server is not None:
            server.terminate()
        if worker is not None and worker.poll() is None:
            worker.terminate()
            try:
                worker.wait(timeout=15)
            except subprocess.TimeoutExpired:
                worker.kill()
                worker.wait()
        status.update(completed_rollouts=len(scores), violations=sum(score < 0 for score in scores))
        write_json(output / "verifai-summary.json", status)
    print(f"Finished {len(scores)} scored rollouts: {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

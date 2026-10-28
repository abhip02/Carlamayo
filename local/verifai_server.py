"""Run a VerifAI server and one persistent CarlaMayo client, scoring collisions."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from importlib.metadata import version
import json
import math
from pathlib import Path
import random
import socket
import subprocess
import time

import numpy as np
import yaml
from dotmap import DotMap
from verifai.monitor import specification_monitor
from verifai.samplers import FeatureSampler
from verifai.server import Server

from local.run_verifai import ROOT, feature_space, rollout_spec, sampling_settings
from local.verifai_client import send_json


def collision_free(summary):
    """Basic specification: no recorded collision during the evaluated episode."""
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
    """

    def __init__(self, sampler, ranges, count, port=8888, timeout=1200, on_sample=None,
                 sampler_type="random"):
        super().__init__(DotMap(sampler=sampler, sample_space=None, sampler_type=sampler_type),
                         specification_monitor(collision_free), DotMap(port=port))
        self.port = self.socket.getsockname()[1]
        self.socket.settimeout(min(0.25, timeout))
        self.ranges, self.count, self.timeout = ranges, count, timeout
        self.on_sample = on_sample
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
        self.current_spec = rollout_spec(sample, self.ranges)
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
        return response["summary"]

    def evaluate_sample(self, sample):
        self.last_result = None
        self.listen()
        try:
            self.send(sample)
            self.last_result = self.receive()
        finally:
            self.close_connection()
        return self.monitor.evaluate(self.last_result)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "local/configs/verifai.yaml")
    parser.add_argument("--sampler", choices=("random", "halton", "ce"), default="random")
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
        count, seed, ranges = sampling_settings(config, args.limit)
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
    random.seed(seed)
    np.random.seed(seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "outputs" / f"verifai-server-{stamp}").resolve()
    output.mkdir(parents=True, exist_ok=False)
    resolved = {**config, "rollouts": [], "expanded_rollouts": [], "sampling_info": {
        "library": "verifai", "version": version("verifai"), "sampler": args.sampler,
        "mode": "server_client", "seed": seed, "sample_count": count,
        "specification": "no_recorded_collision", "passing_score": 1, "failing_score": -1,
    }}
    write_json(output / "config.json", resolved)

    def save_sample(spec):
        resolved["rollouts"].append(spec)
        resolved["expanded_rollouts"].append(spec)
        write_json(output / "config.json", resolved)

    server = None
    worker = None
    results = []
    status = {"status": "error", "requested_rollouts": count, "specification": "no_recorded_collision"}
    try:
        server = RolloutServer(make_sampler(ranges, args.sampler), ranges, count,
                               args.verifai_port, args.timeout, save_sample, args.sampler)
        print(f"VerifAI server listening on 127.0.0.1:{server.port}; "
              f"sampler={args.sampler}, rollouts={count}", flush=True)
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
                          "score": score, "passed": score > 0, "summary": server.last_result,
                          "sample_seconds": timings.sample_time,
                          "evaluation_seconds": timings.simulate_time}
                results.append(result)
                journal.write(json.dumps(result, allow_nan=False) + "\n")
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
        status.update(completed_rollouts=len(results), violations=sum(r["score"] < 0 for r in results))
        write_json(output / "verifai-summary.json", status)
    print(f"Finished {len(results)} scored rollouts: {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

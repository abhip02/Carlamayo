"""Sample initial conditions with VerifAI, then launch one CarlaMayo batch."""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from importlib.metadata import version
import math
from numbers import Real
from pathlib import Path
import random
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ("distance_before_intersection_m", "lateral_offset_m", "speed_mps", "heading_offset_deg")


def sample_config(config, limit=None):
    """Return an explicit rollout config using independent uniform distributions."""
    if not isinstance(config, dict) or "sampling" not in config:
        raise ValueError("Config must contain a sampling block")
    if "rollouts" in config or "initial_conditions" in config:
        raise ValueError("Use sampling instead of rollouts or initial_conditions")
    sampling = config["sampling"]
    if not isinstance(sampling, dict):
        raise ValueError("sampling must be a mapping")
    count, seed = sampling.get("count"), sampling.get("seed")
    if type(count) is not int or count < 1:
        raise ValueError("sampling.count must be a positive integer")
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("sampling.seed must be an integer from 0 through 2**32 - 1")
    if limit is not None:
        if type(limit) is not int or limit < 1:
            raise ValueError("--limit must be a positive integer")
        count = min(count, limit)
    ranges = sampling.get("ranges")
    if not isinstance(ranges, dict) or set(ranges) != set(FIELDS):
        raise ValueError(f"sampling.ranges must contain exactly: {', '.join(FIELDS)}")
    for field, bounds in ranges.items():
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                or any(isinstance(x, bool) or not isinstance(x, Real)
                       or not math.isfinite(x) for x in bounds)):
            raise ValueError(f"{field} must have two finite numeric bounds")
        low, high = bounds
        if low > high:
            raise ValueError(f"{field}: lower bound must not exceed upper bound")
        if field in ("distance_before_intersection_m", "speed_mps") and low < 0:
            raise ValueError(f"{field} must be non-negative")

    # Third-party imports may print banners; stdout is reserved for --sample-only YAML.
    with redirect_stdout(sys.stderr):
        import numpy as np
        from verifai.features import Box, Constant, Feature, FeatureSpace
        from verifai.samplers import FeatureSampler

    py_state, np_state = random.getstate(), np.random.get_state()
    try:
        random.seed(seed)
        np.random.seed(seed)
        space = FeatureSpace({
            field: Feature(Constant(float(low)) if low == high else Box([low, high]))
            for field, (low, high) in ranges.items()
        })
        sampler = FeatureSampler.randomSamplerFor(space)
        rollouts = []
        for _ in range(count):
            point = sampler.nextSample()
            spec = {"name": "verifai"}
            for field in FIELDS:
                value = getattr(point, field)
                spec[field] = float(value if ranges[field][0] == ranges[field][1] else value[0])
            rollouts.append(spec)
    finally:
        random.setstate(py_state)
        np.random.set_state(np_state)

    return {**config, "rollouts": rollouts, "sampling_info": {
        "library": "verifai", "version": version("verifai"), "sampler": "random",
        "seed": seed, "sample_count": count,
    }}


def launch(config, runner_args):
    """Feed all samples to the existing runner, which loads the model once."""
    with tempfile.TemporaryDirectory(prefix="carlamayo-verifai-") as temporary:
        path = Path(temporary) / "sampled.yaml"
        path.write_text(yaml.safe_dump(config, sort_keys=False))
        return subprocess.run(
            ["bash", str(ROOT / "local/run.sh"), "--config", str(path), *runner_args],
            cwd=ROOT,
        ).returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "local/configs/verifai.yaml")
    parser.add_argument("--sample-only", action="store_true",
                        help="Print replayable YAML without connecting to CARLA or loading the model")
    parser.add_argument("--limit", type=int, help="Sample at most this many rollouts")
    parser.add_argument("--duration", type=float, help="Override simulation seconds per rollout")
    parser.add_argument("--host", help="CARLA host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, help="CARLA port (default: 2000)")
    parser.add_argument("--output", type=Path, help="New directory for rollout outputs")
    parser.add_argument("--checkpoint", help="Override the cached checkpoint directory")
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text())
    try:
        resolved = sample_config(config, args.limit)
    except ValueError as exc:
        parser.error(str(exc))
    if args.duration is not None:
        if not math.isfinite(args.duration) or args.duration < 0.1:
            parser.error("--duration must be finite and at least 0.1 seconds")
        resolved["duration_s"] = args.duration
    if args.sample_only:
        print(yaml.safe_dump(resolved, sort_keys=False), end="")
        return 0
    runner_args = []
    for option in ("host", "port", "output", "checkpoint"):
        value = getattr(args, option)
        if value is not None:
            runner_args.extend((f"--{option}", str(value)))
    print(f"VerifAI sampled {len(resolved['rollouts'])} initial states "
          f"with seed {resolved['sampling_info']['seed']}; launching batch.", flush=True)
    return launch(resolved, runner_args)


if __name__ == "__main__":
    raise SystemExit(main())

"""Capture native inputs and replay inference variants without changing defaults.

Run with the model environment after sourcing local/env.sh. Captures include all
four camera histories, pose histories, raw generated tokens and diffusion actions.
All monkey patches are scoped to this separate experimental process.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import copy
import gc
import hashlib
import json
from pathlib import Path
import random
import subprocess
import time

import numpy as np
import torch
import yaml

from local import run_intersection as runner
from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
from module.inference import extract_trajectory_samples
from module.trajectory_follower import planned_speeds


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


class Probe:
    def __init__(self, model, output):
        self.model, self.output = model, output
        self.options, self.context, self.trace = {}, {}, {}
        self.original_generate = model.vlm.generate
        self.original_decode = model.action_space.action_to_traj
        self.original_sample = model.sample_trajectories_from_data_with_vlm_rollout
        self.original_cfg_sample = model.sample_trajectories_from_data_with_vlm_rollout_cfg_nav
        self.original_flow_sample = model.diffusion.sample
        model.vlm.generate = self.generate
        model.action_space.action_to_traj = self.decode
        model.sample_trajectories_from_data_with_vlm_rollout = self.sample
        model.sample_trajectories_from_data_with_vlm_rollout_cfg_nav = self.cfg_sample
        model.diffusion.sample = self.flow_sample

    def close(self):
        self.model.vlm.generate = self.original_generate
        self.model.action_space.action_to_traj = self.original_decode
        self.model.sample_trajectories_from_data_with_vlm_rollout = self.original_sample
        self.model.sample_trajectories_from_data_with_vlm_rollout_cfg_nav = self.original_cfg_sample
        self.model.diffusion.sample = self.original_flow_sample

    def flow_sample(self, *args, **kwargs):
        self.trace["diffusion_rng_fingerprint"] = [
            hashlib.sha256(torch.cuda.get_rng_state(i).numpy().tobytes()).hexdigest()[:16]
            for i in range(torch.cuda.device_count())]
        if "flow_seed" not in self.options:
            return self.original_flow_sample(*args, **kwargs)
        with torch.random.fork_rng(devices=list(range(torch.cuda.device_count()))):
            torch.manual_seed(self.options["flow_seed"])
            torch.cuda.manual_seed_all(self.options["flow_seed"])
            return self.original_flow_sample(*args, **kwargs)

    def sample(self, *args, **kwargs):
        return self.sample_with_options(self.original_sample, *args, **kwargs)

    def cfg_sample(self, *args, **kwargs):
        return self.sample_with_options(self.original_cfg_sample, *args, **kwargs)

    def sample_with_options(self, original, *args, **kwargs):
        kwargs = dict(kwargs)
        diffusion = dict(kwargs.get("diffusion_kwargs", {}))
        for option, key in (("flow_steps", "inference_step"), ("flow_temperature", "temperature")):
            if option in self.options:
                diffusion[key] = self.options[option]
        kwargs["diffusion_kwargs"] = diffusion
        if "max_generation_length" in self.options:
            kwargs["max_generation_length"] = self.options["max_generation_length"]
        return original(*args, **kwargs)

    def generate(self, *args, **kwargs):
        if self.options.get("greedy_text"):
            kwargs["generation_config"] = copy.deepcopy(kwargs["generation_config"])
            kwargs["generation_config"].do_sample = False
        if self.options.get("guard_trajectory_format"):
            from transformers import LogitsProcessorList
            marker = self.model.tokenizer.convert_tokens_to_ids("<|traj_future_start|>")
            vocab = self.model.tokenizer.get_vocab()
            forbidden = [vocab[token] for token in (
                "<|answer_start|>", "<|answer_end|>", "<|im_end|>", "<|endoftext|>") if token in vocab]

            class RequireTrajectoryMarker:
                def __call__(self, input_ids, scores):
                    for row in range(input_ids.shape[0]):
                        if not torch.any(input_ids[row] == marker):
                            scores[row, forbidden] = -torch.inf
                    return scores

            kwargs["logits_processor"] = LogitsProcessorList(
                [*kwargs.get("logits_processor", []), RequireTrajectoryMarker()])
        result = self.original_generate(*args, **kwargs)
        prompt_length = kwargs["input_ids"].shape[-1]
        tokens = result.sequences[0, prompt_length:].detach().cpu().tolist()
        marker = self.model.tokenizer.convert_tokens_to_ids("<|traj_future_start|>")
        self.trace.update(generated_token_ids=tokens, generated_tokens=len(tokens),
                          trajectory_marker_present=marker in tokens,
                          raw_generated_text=self.model.tokenizer.decode(tokens, skip_special_tokens=False))
        return result

    def decode(self, action, history_xyz, history_rot, t0_states=None):
        space = self.model.action_space
        states = t0_states or space.estimate_t0_states(history_xyz, history_rot)
        estimated = states["v"]
        used = estimated
        if self.options.get("initial_speed") == "measured":
            used = torch.full_like(estimated, self.context["measured_signed_speed_mps"])
        elif self.options.get("initial_speed") == "nonnegative":
            used = estimated.clamp(min=0)
        acceleration = action[..., 0] * space.accel_std + space.accel_mean
        velocity = used.unsqueeze(-1) + torch.cumsum(acceleration * space.dt, dim=-1)
        self.trace.update(
            decoder_estimated_speed_mps=float(estimated.flatten()[0]),
            decoder_used_speed_mps=float(used.flatten()[0]),
            sampled_action=action.detach().float().cpu().tolist(),
            acceleration_mps2=acceleration.detach().float().cpu().flatten().tolist(),
            signed_velocity_mps=velocity.detach().float().cpu().flatten().tolist(),
        )
        return self.original_decode(action, history_xyz, history_rot, {**states, "v": used})

    def evaluate(self, adapter, data, context, instruction, options, label, capture=False,
                 capture_failures=False):
        self.options, self.context, self.trace = options, context, {}
        seed = options.get("seed", context.get("seed", 42))
        random.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
        if capture:
            capture_path = self.output / "inputs" / (label + ".pt")
            capture_path.parent.mkdir(exist_ok=True)
            torch.save({"data": {key: value.detach().cpu().clone() for key, value in data.items()},
                        "context": context, "instruction": instruction}, capture_path)
        started = time.monotonic()
        for device in range(torch.cuda.device_count()):
            torch.cuda.reset_peak_memory_stats(device)
        from torch.nn.attention import SDPBackend, sdpa_kernel
        attention_context = sdpa_kernel(SDPBackend.MATH) if options.get("sdpa_math") else nullcontext()
        with attention_context:
            prediction, extra = Alpamayo15Adapter.run_inference(
                adapter, self.model, adapter.probe_processor, data,
                navigation_text=options.get("instruction", instruction),
                navigation_weight=options.get("navigation_weight", 1.0))
        trajectory = extract_trajectory_samples(prediction)[0]
        speeds = planned_speeds(trajectory)
        result = {"label": label, "context": context, "options": options, "seed": seed,
                  "instruction": options.get("instruction", instruction),
                  "inference_seconds": time.monotonic() - started,
                  "peak_allocated_gib": [torch.cuda.max_memory_allocated(i) / 2**30 for i in range(torch.cuda.device_count())],
                  "raw_target_speed_mps": float(speeds[10:15].mean()),
                  "first_second_speed_mps": float(speeds[:10].mean()),
                  "end_forward_m": float(trajectory[-1, 0]),
                  "end_left_m": float(trajectory[-1, 1]),
                  "reverse_velocity_steps": sum(v < -0.05 for v in self.trace["signed_velocity_mps"]),
                  "trajectory_ego": trajectory.tolist(),
                  "reasoning": adapter.extract_cot_text(extra), **self.trace}
        if not capture and capture_failures and not self.trace["trajectory_marker_present"]:
            capture_path = self.output / "inputs" / (label + ".pt")
            capture_path.parent.mkdir(exist_ok=True)
            torch.save({"data": {key: value.detach().cpu().clone() for key, value in data.items()},
                        "context": context, "instruction": instruction}, capture_path)
        write_json(self.output / (label + ".json"), result)
        print(json.dumps({key: result[key] for key in (
            "label", "raw_target_speed_mps", "decoder_estimated_speed_mps",
            "decoder_used_speed_mps", "end_forward_m", "end_left_m",
            "reverse_velocity_steps", "trajectory_marker_present", "inference_seconds")}), flush=True)
        return prediction, extra


def capture(args, output):
    config = yaml.safe_load(args.config.read_text())
    config["duration_s"] = args.duration
    options = json.loads(args.options.read_text()) if args.options else {}
    config["instruction"] = options.get("instruction", config.get("instruction"))
    specs = runner.rollout_specs(config)
    write_json(output / "config.json", config)
    active = {}
    original_adapter, original_connect = runner.Alpamayo15Adapter, runner.connect_world

    def connect(*values):
        client, world = original_connect(*values)
        active["world"] = world
        return client, world

    class RecordingAdapter(Alpamayo15Adapter):
        def load_model(self, *values, **kwargs):
            model, processor = super().load_model(*values, **kwargs)
            self.probe_processor = processor
            active["probe"] = Probe(model, output)
            self.index = 0
            return model, processor

        def run_inference(self, model, processor, data, **kwargs):
            world = active["world"]
            snapshot = world.get_snapshot()
            vehicle = next(actor for actor in world.get_actors().filter("vehicle.*")
                           if actor.attributes.get("role_name") == "carlamayo_intersection")
            actor = snapshot.find(vehicle.id)
            transform, velocity = actor.get_transform(), actor.get_velocity()
            forward = transform.get_forward_vector()
            context = {"index": self.index, "time_s": self.index * .1,
                       "snapshot_frame": snapshot.frame, "seed": config.get("seed", 42),
                       "measured_signed_speed_mps": velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z,
                       "measured_speed_mps": velocity.length()}
            label = f"frame_{self.index:03d}"
            result = active["probe"].evaluate(self, data, context, kwargs.get("navigation_text"),
                                              options, label, self.index in args.capture_frames,
                                              capture_failures=True)
            self.index += 1
            return result

    runner.Alpamayo15Adapter, runner.connect_world = RecordingAdapter, connect
    try:
        with runner.rollout_session(args, config, output) as episode:
            summary = episode(specs[0], output / "000_verifai")
            write_json(output / "batch-summary.json", [summary])
    finally:
        if "probe" in active:
            active["probe"].close()
        runner.Alpamayo15Adapter, runner.connect_world = original_adapter, original_connect


def replay(args, output):
    adapter = Alpamayo15Adapter()
    model, processor = runner.load_sharded_model(args, adapter)
    adapter.probe_processor = processor
    probe = Probe(model, output)
    variants = json.loads(args.variants.read_text()) if args.variants else {
        "baseline": {}, "measured_speed": {"initial_speed": "measured"},
        "nonnegative_speed": {"initial_speed": "nonnegative"},
        "flow_30_steps": {"flow_steps": 30}, "flow_temp_06": {"flow_temperature": 0.6},
        "seed_0": {"seed": 0}, "seed_43": {"seed": 43},
        "short_navigation": {"instruction": "Continue straight through the intersection."},
        "no_navigation": {"instruction": ""},
        "guard_format": {"guard_trajectory_format": True},
        "greedy_text": {"greedy_text": True},
        "tokens_512": {"max_generation_length": 512},
    }
    try:
        guidance_oom = False
        for source in args.inputs:
            saved = torch.load(source, map_location="cpu", weights_only=True)
            for name, options in variants.items():
                label = source.stem + "_" + name
                if guidance_oom and options.get("navigation_weight", 1) != 1:
                    write_json(output / (label + ".json"), {"label": label, "status": "skipped",
                               "reason": "A preceding guidance experiment exceeded available GPU memory"})
                    continue
                try:
                    probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"], options, label)
                except torch.cuda.OutOfMemoryError as exc:
                    write_json(output / (label + ".json"), {"label": label, "status": "oom", "error": str(exc)})
                    guidance_oom = guidance_oom or options.get("navigation_weight", 1) != 1
                    print(f"{label}: GPU memory exhausted; logged and skipping repeated guidance attempts", flush=True)
                except Exception as exc:
                    write_json(output / (label + ".json"), {"label": label, "status": "error",
                               "error": f"{type(exc).__name__}: {exc}"})
                    # An illegal CUDA access poisons the process; do not continue.
                    raise
                gc.collect()
                torch.cuda.empty_cache()
    finally:
        probe.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "replay"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--duration", type=float, default=6)
    parser.add_argument("--capture-frames", nargs="+", type=int, default=[0, 15, 38, 39, 57, 58])
    parser.add_argument("--inputs", nargs="+", type=Path)
    parser.add_argument("--variants", type=Path)
    parser.add_argument("--options", type=Path, help="Opt-in capture-run variant settings")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--checkpoint")
    args = parser.parse_args()
    if args.mode == "capture" and not args.config:
        parser.error("capture requires --config")
    if args.mode == "replay" and not args.inputs:
        parser.error("replay requires --inputs")
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "experiment.json", {
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=runner.ROOT, text=True).strip(),
        "arguments": {key: str(value) for key, value in vars(args).items()},
        "source_sha256": {name: hashlib.sha256((runner.ROOT / name).read_bytes()).hexdigest()
                          for name in ["module/config.py", "module/trajectory_follower.py", __file__]},
    })
    (capture if args.mode == "capture" else replay)(args, args.output)


if __name__ == "__main__":
    main()

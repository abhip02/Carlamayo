"""Capture native inputs and replay inference variants without changing defaults.

Run with the model environment after sourcing local/env.sh. Captures include all
four camera histories, pose histories, raw generated tokens and diffusion actions.
All monkey patches are scoped to this separate experimental process.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import copy
import fcntl
import gc
import hashlib
import json
import math
from pathlib import Path
import random
import re
import subprocess
import sys
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


def acquire_experiment_lock():
    """These probes share two model GPUs; reject overlapping launches early."""
    path = runner.ROOT / ".runtime" / "driving-probe.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    lock = path.open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("Another inference probe is running; wait for it to exit before launching this one.") from None
    return lock


class Probe:
    def __init__(self, model, output):
        from alpamayo1_5 import helper
        self.model, self.output = model, output
        self.options, self.context, self.trace = {}, {}, {}
        self.previous_plan = None
        self.visual_signal = None
        self.visual_signal_index = -1000
        self.original_generate = model.vlm.generate
        self.original_decode = model.action_space.action_to_traj
        self.original_sample = model.sample_trajectories_from_data_with_vlm_rollout
        self.original_cfg_sample = model.sample_trajectories_from_data_with_vlm_rollout_cfg_nav
        self.original_flow_sample = model.diffusion.sample
        self.original_create_message = helper.create_message
        helper.create_message = self.create_message
        model.vlm.generate = self.generate
        model.action_space.action_to_traj = self.decode
        model.sample_trajectories_from_data_with_vlm_rollout = self.sample
        model.sample_trajectories_from_data_with_vlm_rollout_cfg_nav = self.cfg_sample
        model.diffusion.sample = self.flow_sample

    def close(self):
        from alpamayo1_5 import helper
        helper.create_message = self.original_create_message
        self.model.vlm.generate = self.original_generate
        self.model.action_space.action_to_traj = self.original_decode
        self.model.sample_trajectories_from_data_with_vlm_rollout = self.original_sample
        self.model.sample_trajectories_from_data_with_vlm_rollout_cfg_nav = self.original_cfg_sample
        self.model.diffusion.sample = self.original_flow_sample

    def create_message(self, *args, **kwargs):
        messages = self.original_create_message(*args, **kwargs)
        task = messages[-2]["content"][-1]
        if self.options.get("route_in_user_text"):
            task["text"] += " Follow this navigation instruction: " + (kwargs.get("nav_text") or "")
        if "prompt_hint" in self.options:
            task["text"] += " " + self.options["prompt_hint"]
        if self.options.get("prompt_style") == "trajectory_only":
            task["text"] = task["text"].replace(
                "output the chain-of-thought reasoning of the driving process, then output the future trajectory.",
                "output the future trajectory.")
            messages[-1]["content"][0]["text"] = "<|traj_future_start|>"
        if "reasoning_prefix" in self.options:
            messages[-1]["content"][0]["text"] = "<|cot_start|>" + self.options["reasoning_prefix"]
        if "completed_reasoning" in self.options:
            # Diagnostic: condition the motion expert on a supplied intent.
            # This text is not a model-generated explanation or perception result.
            messages[-1]["content"][0]["text"] = (
                "<|cot_start|>" + self.options["completed_reasoning"] +
                "<|cot_end|><|traj_future_start|>")
        self.trace["assistant_prefix"] = messages[-1]["content"][0]["text"]
        self.trace["user_task_text"] = task["text"]
        return messages

    def flow_sample(self, *args, **kwargs):
        self.trace["diffusion_rng_fingerprint"] = [
            hashlib.sha256(torch.cuda.get_rng_state(i).numpy().tobytes()).hexdigest()[:16]
            for i in range(torch.cuda.device_count())]
        from torch.nn.attention import SDPBackend, sdpa_kernel
        attention = sdpa_kernel(SDPBackend.MATH) if self.options.get("expert_sdpa_math") else nullcontext()
        with attention:
            if "flow_seed" not in self.options:
                self.trace["diffusion_rng_fingerprint_used"] = self.trace["diffusion_rng_fingerprint"]
                return self.original_flow_sample(*args, **kwargs)
            with torch.random.fork_rng(devices=list(range(torch.cuda.device_count()))):
                torch.manual_seed(self.options["flow_seed"])
                torch.cuda.manual_seed_all(self.options["flow_seed"])
                self.trace["diffusion_rng_fingerprint_used"] = [
                    hashlib.sha256(torch.cuda.get_rng_state(i).numpy().tobytes()).hexdigest()[:16]
                    for i in range(torch.cuda.device_count())]
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
        if "num_samples" in self.options:
            kwargs["num_traj_samples"] = self.options["num_samples"]
        if "text_temperature" in self.options:
            kwargs["temperature"] = self.options["text_temperature"]
        if "completed_reasoning" in self.options:
            # Prefill already includes the trajectory marker. One generated token
            # is enough to obtain its KV cache; StopAfterEOS only notices newly
            # generated markers and would otherwise generate irrelevant text.
            kwargs["max_generation_length"] = 1
        return original(*args, **kwargs)

    def generate(self, *args, **kwargs):
        # Transformers otherwise treats the VQA value 1 as a default and can
        # inherit the preceding trajectory call's num_return_sequences=4.
        kwargs.setdefault("num_return_sequences", kwargs["generation_config"].num_return_sequences)
        if self.options.get("greedy_text"):
            kwargs["generation_config"] = copy.deepcopy(kwargs["generation_config"])
            kwargs["generation_config"].do_sample = False
        # VQA has its own <answer_start> / <answer_end> protocol. Applying the
        # trajectory guard there blocks answer termination and causes rambling.
        answer_start = self.model.tokenizer.convert_tokens_to_ids("<|answer_start|>")
        is_vqa = bool(torch.all(kwargs["input_ids"][:, -1] == answer_start))
        if self.options.get("guard_trajectory_format") and not is_vqa:
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
        batch_tokens = result.sequences[:, prompt_length:].detach().cpu().tolist()
        marker = self.model.tokenizer.convert_tokens_to_ids("<|traj_future_start|>")
        self.trace.update(generated_token_ids=tokens, generated_tokens=len(tokens),
                          trajectory_marker_present=bool(torch.any(result.sequences[0] == marker)),
                          trajectory_marker_generated=marker in tokens,
                          raw_generated_text=self.model.tokenizer.decode(tokens, skip_special_tokens=False))
        self.trace["candidate_generated_tokens"] = batch_tokens
        self.trace["candidate_markers"] = [marker in sequence for sequence in result.sequences.detach().cpu().tolist()]
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
            candidate_velocities=velocity.detach().float().cpu().tolist(),
            candidate_accelerations=acceleration.detach().float().cpu().tolist(),
        )
        decoded = self.original_decode(action, history_xyz, history_rot, {**states, "v": used})
        rotations = decoded[1]
        self.trace["candidate_yaw_rad"] = torch.atan2(
            rotations[..., 1, 0].float(), rotations[..., 0, 0].float()
        ).detach().cpu().reshape(-1, rotations.shape[-3]).tolist()
        return decoded

    def evaluate(self, adapter, data, context, instruction, options, label, capture=False,
                 capture_failures=False):
        source_data = data
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
        if "camera_projection" in options:
            from local.experiments.camera_projection import project_history
            if context.get("render_projection") != "wide_pinhole":
                raise ValueError("Lens experiments require captures rendered with the experimental wide rig")
            data = project_history(data, options["camera_projection"])
        if "camera_subset" in options:
            indices = data["camera_indices"].tolist()
            selected = [indices.index(index) for index in options["camera_subset"]]
            data = {**data, "image_frames": data["image_frames"][selected],
                    "camera_indices": data["camera_indices"][selected]}
        self.trace["input_camera_indices"] = data["camera_indices"].tolist()
        self.effective_data = data
        if options.get("verify_signal"):
            index = context["index"]
            if index - self.visual_signal_index >= options.get("signal_check_interval", 5):
                question = "What color is the traffic light facing the ego vehicle? Answer only red, yellow, green, or unknown."
                with torch.random.fork_rng(devices=list(range(torch.cuda.device_count()))):
                    torch.manual_seed(42)
                    torch.cuda.manual_seed_all(42)
                    answer = adapter.run_vqa(self.model, adapter.probe_processor, data, question)
                answer_text = adapter.extract_answer_text(answer).strip()
                colors = set(re.findall(r"\b(red|yellow|green)\b", answer_text.lower()))
                self.visual_signal = next(iter(colors)) if len(colors) == 1 else "unknown"
                self.visual_signal_answer = answer_text
                self.visual_signal_index = index
            self.trace["visual_signal"] = self.visual_signal
            self.trace["visual_signal_answer"] = self.visual_signal_answer
            self.trace["visual_signal_check_index"] = self.visual_signal_index
            if self.visual_signal in ("red", "yellow"):
                stop_option = ("completed_reasoning" if options.get("complete_visual_stop_intent")
                               else "reasoning_prefix")
                self.options = {**options, stop_option:
                                f"Stop and remain stopped for the {self.visual_signal} traffic light"}
        for device in range(torch.cuda.device_count()):
            torch.cuda.reset_peak_memory_stats(device)
        from torch.nn.attention import SDPBackend, sdpa_kernel
        attention_context = sdpa_kernel(SDPBackend.MATH) if options.get("sdpa_math") else nullcontext()
        with attention_context:
            prediction, extra = Alpamayo15Adapter.run_inference(
                adapter, self.model, adapter.probe_processor, data,
                navigation_text=options.get("instruction", instruction),
                navigation_weight=options.get("navigation_weight", 1.0))
        candidates = extract_trajectory_samples(prediction)
        candidate_texts = {key: value.tolist() if isinstance(value, np.ndarray) else value
                           for key, value in extra.items()} if len(candidates) > 1 else None
        chosen = 0
        if options.get("select_route_continuity"):
            if not context.get("closed_loop"):
                raise ValueError("Route-continuity selection requires a live capture with pose and route context")
            from local.experiments.candidate_selection import select_candidate
            chosen, self.previous_plan, self.trace["selection"] = select_candidate(
                candidates, context, self.previous_plan,
                prefer_stop=bool(options.get("prefer_stop_on_red") and self.visual_signal in ("red", "yellow")))
            prediction = prediction[:, :, chosen:chosen + 1]
            extra = {key: value[..., chosen:chosen + 1] if isinstance(value, np.ndarray) else value
                     for key, value in extra.items()}
        trajectory = candidates[chosen]
        chosen_tokens = self.trace["candidate_generated_tokens"][chosen]
        self.trace.update(generated_token_ids=chosen_tokens, generated_tokens=len(chosen_tokens),
                          trajectory_marker_present=self.trace["candidate_markers"][chosen],
                          trajectory_marker_generated=self.model.tokenizer.convert_tokens_to_ids("<|traj_future_start|>") in chosen_tokens,
                          raw_generated_text=self.model.tokenizer.decode(chosen_tokens, skip_special_tokens=False),
                          signed_velocity_mps=self.trace["candidate_velocities"][chosen],
                          acceleration_mps2=self.trace["candidate_accelerations"][chosen])
        speeds = planned_speeds(trajectory)
        result = {"label": label, "context": context, "options": options, "seed": seed,
                  "instruction": options.get("instruction", instruction),
                  "inference_seconds": time.monotonic() - started,
                  "peak_allocated_gib": [torch.cuda.max_memory_allocated(i) / 2**30 for i in range(torch.cuda.device_count())],
                  "raw_target_speed_mps": float(speeds[10:15].mean()),
                  "controller_preview_speed_mps": float(speeds[
                      round(runner.defaults.PID_SPEED_LEAD_SEC / .1):
                      round((runner.defaults.PID_SPEED_LEAD_SEC + runner.defaults.PID_SPEED_PREVIEW_SEC) / .1)
                  ].mean()),
                  "first_second_speed_mps": float(speeds[:10].mean()),
                  "end_forward_m": float(trajectory[-1, 0]),
                  "end_left_m": float(trajectory[-1, 1]),
                  "reverse_velocity_steps": sum(v < -0.05 for v in self.trace["signed_velocity_mps"]),
                  "trajectory_ego": trajectory.tolist(),
                  "candidate_trajectories": candidates.tolist() if len(candidates) > 1 else None,
                  "candidate_texts": candidate_texts,
                  "reasoning": adapter.extract_cot_text(extra), **self.trace}
        if not capture and capture_failures and not self.trace["trajectory_marker_present"]:
            capture_path = self.output / "inputs" / (label + ".pt")
            capture_path.parent.mkdir(exist_ok=True)
            torch.save({"data": {key: value.detach().cpu().clone() for key, value in source_data.items()},
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
    original_tick = runner.CARLAInterface.tick
    original_initialize = runner.initialize_ego
    original_apply_control = runner.CARLAInterface.apply_control
    snapshot_waits = []
    original_speed_lead = runner.defaults.PID_SPEED_LEAD_SEC
    if "controller_speed_lead_s" in options:
        runner.defaults.PID_SPEED_LEAD_SEC = options["controller_speed_lead_s"]

    def connect(*values):
        client, world = original_connect(*values)
        active["world"] = world
        return client, world

    def synchronized_tick(interface):
        from local.experiments.frame_sync import wait_for_frame
        frame = original_tick(interface)
        immediate = interface.world.get_snapshot().frame
        started = time.monotonic()
        wait_for_frame(interface.world, frame)
        snapshot_waits.append({"frame": frame, "immediate_snapshot_frame": immediate,
                               "wait_seconds": time.monotonic() - started})
        return frame

    def acknowledged_control(interface, steering, throttle, brake):
        import carla
        control = carla.VehicleControl(steer=float(steering), throttle=float(throttle), brake=float(brake))
        response = interface.client.apply_batch_sync([
            carla.command.ApplyVehicleControl(interface.ego_vehicle.id, control)], False)[0]
        if response.error:
            raise RuntimeError(response.error)

    class RecordingAdapter(Alpamayo15Adapter):
        if "camera_projection" in options:
            from local.experiments.camera_projection import wide_render_rig
            source_camera_configs = wide_render_rig()

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
            applied = vehicle.get_control()
            bounds = vehicle.bounding_box
            context = {"index": self.index, "time_s": self.index * .1,
                       "closed_loop": True,
                       "world_pose": {"x": transform.location.x, "y": transform.location.y, "z": transform.location.z,
                                      "yaw": transform.rotation.yaw, "pitch": transform.rotation.pitch, "roll": transform.rotation.roll},
                       "route_entry": config["junction_entry"],
                       "vehicle_bounds": {
                           "blueprint": vehicle.type_id,
                           "location": {key: getattr(bounds.location, key) for key in ("x", "y", "z")},
                           "extent": {key: getattr(bounds.extent, key) for key in ("x", "y", "z")},
                           "rotation": {key: getattr(bounds.rotation, key) for key in ("pitch", "yaw", "roll")}},
                       "render_projection": "wide_pinhole" if "camera_projection" in options else "native_pinhole",
                       "snapshot_frame": snapshot.frame, "seed": config.get("seed", 42),
                       "applied_control": {"steer": applied.steer, "throttle": applied.throttle,
                                           "brake": applied.brake, "reverse": applied.reverse},
                       "signal_states": {str(light.id): str(light.get_state()) for light in active.get("signals", [])},
                       "measured_signed_speed_mps": velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z,
                       "measured_speed_mps": velocity.length()}
            label = f"frame_{self.index:03d}"
            result = active["probe"].evaluate(self, data, context, kwargs.get("navigation_text"),
                                              options, label, self.index in args.capture_frames,
                                              capture_failures=True)
            self.index += 1
            return result

    runner.Alpamayo15Adapter, runner.connect_world = RecordingAdapter, connect
    if options.get("wait_for_snapshot"):
        runner.CARLAInterface.tick = synchronized_tick
    if options.get("acknowledged_preroll"):
        from local.experiments.acknowledged_preroll import initialize_ego
        runner.initialize_ego = initialize_ego
    if options.get("acknowledged_controls"):
        runner.CARLAInterface.apply_control = acknowledged_control
    try:
        with runner.rollout_session(args, config, output) as episode:
            entry = config["junction_entry"]
            # Record the default all-green scene as well as explicit variants.
            # The selector still reads only the separate image-based answer.
            active["signals"] = [
                light for light in active["world"].get_actors().filter("traffic.traffic_light")
                if math.hypot(light.get_location().x - entry["x"],
                              light.get_location().y - entry["y"]) < 50]
            if options.get("traffic_layout") == "ego_green":
                import carla
                entry = config["junction_entry"]
                world = active["world"]
                selected = []
                for light in world.get_actors().filter("traffic.traffic_light"):
                    for waypoint in light.get_stop_waypoints():
                        transform = waypoint.transform
                        difference = (transform.rotation.yaw - entry["yaw"] + 180) % 360 - 180
                        if abs(difference) < 15 and math.hypot(
                                transform.location.x - entry["x"], transform.location.y - entry["y"]) < 8:
                            selected.append(light)
                            break
                if not selected:
                    raise RuntimeError("Could not identify the approach traffic signal")
                group = {light.id: light for chosen in selected for light in chosen.get_group_traffic_lights()}
                for light in group.values():
                    light.set_state(carla.TrafficLightState.Red)
                    light.freeze(True)
                for light in selected:
                    light.set_state(carla.TrafficLightState.Green)
                active["signals"] = list(group.values())
                write_json(output / "signal-layout.json", {
                    "green_signal_ids": [light.id for light in selected],
                    "red_signal_ids": sorted(set(group) - {light.id for light in selected})})
            if "traffic_light_state" in options:
                import carla
                state = getattr(carla.TrafficLightState, options["traffic_light_state"])
                lights = active["world"].get_actors().filter("traffic.traffic_light")
                for light in lights:
                    light.set_state(state)
                    light.freeze(True)
                entry = config["junction_entry"]
                active["signals"] = [light for light in lights if math.hypot(
                    light.get_location().x - entry["x"], light.get_location().y - entry["y"]) < 40]
            summary = episode(specs[0], output / "000_verifai")
            write_json(output / "batch-summary.json", [summary])
    finally:
        if "probe" in active:
            active["probe"].close()
        runner.Alpamayo15Adapter, runner.connect_world = original_adapter, original_connect
        runner.CARLAInterface.tick = original_tick
        runner.initialize_ego = original_initialize
        runner.CARLAInterface.apply_control = original_apply_control
        runner.defaults.PID_SPEED_LEAD_SEC = original_speed_lead
        if snapshot_waits:
            write_json(output / "snapshot-waits.json", snapshot_waits)


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
                    # A replay variant is an independent frozen-input experiment.
                    probe.previous_plan = None
                    probe.visual_signal_index = -1000
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
            if args.vqa_question:
                probe.options = {}
                torch.manual_seed(42)
                torch.cuda.manual_seed_all(42)
                answer_data = adapter.run_vqa(model, processor, probe.effective_data, args.vqa_question)
                answer = adapter.extract_answer_text(answer_data)
                write_json(output / (source.stem + "_vqa.json"), {
                    "question": args.vqa_question, "answer": answer,
                    "raw": {key: value.tolist() if isinstance(value, np.ndarray) else value
                            for key, value in answer_data.items()},
                    "input_options": options,
                    "note": "Uses the last replay variant's camera preprocessing; VQA has no ego-history input."})
                print(f"VQA {source.stem}: {answer}", flush=True)
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
    parser.add_argument("--vqa-question", help="Ask one visual question after each input's replay variants")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--checkpoint")
    args = parser.parse_args()
    if args.mode == "capture" and not args.config:
        parser.error("capture requires --config")
    if args.mode == "replay" and not args.inputs:
        parser.error("replay requires --inputs")
    with acquire_experiment_lock():
        args.output.mkdir(parents=True, exist_ok=False)
        sources = {name: (runner.ROOT / name).read_bytes() for name in (
            "module/config.py", "module/trajectory_follower.py",
            "local/experiments/inference_probe.py", "local/experiments/camera_projection.py",
            "local/experiments/candidate_selection.py", "local/experiments/frame_sync.py",
            "local/experiments/acknowledged_preroll.py", "local/experiments/validated_probe.py",
            "local/experiments/validated_selection.py", "local/experiments/retained_probe.py",
            "local/experiments/proactive_probe.py", "local/experiments/tracking_probe.py",
            "local/experiments/tracking_forecast.py", "local/experiments/exit_cache.py")}
        for name, content in sources.items():
            snapshot = args.output / "source_snapshot" / name
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            snapshot.write_bytes(content)
        write_json(args.output / "experiment.json", {
            "entrypoint": sys.argv[0],
            "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=runner.ROOT, text=True).strip(),
            "arguments": {key: str(value) for key, value in vars(args).items()},
            "options": json.loads(args.options.read_text()) if args.options else None,
            "variants": json.loads(args.variants.read_text()) if args.variants else None,
            "source_sha256": {name: hashlib.sha256(content).hexdigest()
                              for name, content in sources.items()},
        })
        (capture if args.mode == "capture" else replay)(args, args.output)


if __name__ == "__main__":
    main()

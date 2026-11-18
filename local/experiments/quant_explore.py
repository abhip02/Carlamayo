"""Maneuver replay and paired quantization sensitivity, without a controller.

The A8/A4 variants quantize/dequantize inputs to packed NF4 linear layers.
They emulate numerical error, not native integer execution or activation storage.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shutil
import time

import carla
import numpy as np
import torch

from local.experiments import model_score_benchmark as base

ROOT = base.ROOT
OLD = ROOT / "outputs/model-score-benchmark-20261105"
VARIANTS = ("bf16", "nf4", "nf4_a8", "nf4_a4")


def anchors(rows):
    """Three deterministic anchors with full 1.5s history and 3s future."""
    times = np.array([r["time_s"] for r in rows])
    eligible = np.flatnonzero((times >= times[0] + 1.5 - 1e-6) &
                             (times <= times[-1] - 3.1 + 1e-6))
    if len(eligible) < 3:
        raise ValueError("Need three distinct observations with complete history/future")
    return eligible[np.rint(np.linspace(0, len(eligible) - 1, 3)).astype(int)].tolist()


def activation_qdq(x, bits):
    """Symmetric last-dimension per-token absmax quantization, FP32 scaling."""
    if bits not in (4, 8):
        raise ValueError("Supported activation bitwidths: 4, 8")
    qmax = 2 ** (bits - 1) - 1
    value = x.float()
    scale = value.abs().amax(dim=-1, keepdim=True).clamp_min(1e-12) / qmax
    return ((value / scale).round().clamp(-qmax, qmax) * scale).to(x.dtype)


def install_activation_hooks(model, bits):
    import bitsandbytes as bnb
    handles, names = [], []
    def hook(module, args):
        return (activation_qdq(args[0], bits), *args[1:])
    for name, module in model.named_modules():
        if isinstance(module, bnb.nn.Linear4bit):
            handles.append(module.register_forward_pre_hook(hook))
            names.append(name)
    if len(names) != 624:
        raise ValueError(f"Unexpected NF4 layer count: {len(names)}")
    return handles, names


def score_plan(trajectory, yaw, context, route, reference, mission):
    # Use the previously validated geometry without modifying its frozen source.
    old_horizons = base.HORIZONS
    try:
        base.HORIZONS = (1, 3)
        return base.trajectory_scores(trajectory, yaw, context, route, reference, mission)
    finally:
        base.HORIZONS = old_horizons


def capture(output):
    from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
    from module.carla_interface import CARLAInterface
    from module.open_loop_dataset import build_ego_history
    from local.experiments.projected_probe import route_state

    output.mkdir(parents=True, exist_ok=False)
    old = json.loads((OLD / "protocol.json").read_text())
    scenarios = json.loads((ROOT / "local/experiments/presets/ten-driving-scenes.json").read_text())["scenarios"]
    adapter = Alpamayo15Adapter()
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(120)
    world = client.get_world()
    if world.get_map().name.split("/")[-1] != "Town03":
        world = client.load_world("Town03")
    original_settings, original_weather = world.get_settings(), world.get_weather()
    lights = [(a, a.get_state(), a.is_frozen()) for a in world.get_actors().filter("traffic.traffic_light")]
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, .1
    world.apply_settings(settings)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    cases = []
    try:
        for original, scenario in zip(old["cases"], scenarios, strict=True):
            source = Path(original["source"])
            folder = source.parents[1]
            with (folder / "000_verifai/ego.csv").open() as stream:
                rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]
            route = json.loads((OLD / "inputs" / original["name"] / "route.json").read_text())
            selected = anchors(rows)
            expected = carla.TrafficLightState.Red if original["mission"] else carla.TrafficLightState.Green
            for light, _, _ in lights:
                light.set_state(expected)
                light.freeze(True)
            interface = CARLAInterface(adapter.source_camera_configs)
            interface.client, interface.world = client, world
            try:
                bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
                bp.set_attribute("role_name", "quant_explore_pose_replay")
                spawn = base.pose_transform(rows[selected[0] - 3])
                spawn.location.z += .5
                interface.ego_vehicle = world.spawn_actor(bp, spawn)
                interface.ego_vehicle.set_simulate_physics(False)
                interface.setup_cameras()
                poses = {str(i): row for i, row in enumerate(rows)}
                phase = "approach"
                instructions = []
                for row in rows:
                    _, phase, instruction = route_state({"world_pose": row}, route, scenario["route"], phase)
                    instructions.append(instruction)
                for position, anchor in enumerate(selected):
                    images = []
                    replay_rows = [anchor - 3] * 5 + list(range(anchor - 3, anchor + 1))
                    for j, index in enumerate(replay_rows):
                        target = base.pose_transform(rows[index])
                        result = client.apply_batch_sync([
                            carla.command.ApplyTransform(interface.ego_vehicle.id, target)], False)
                        if result[0].error:
                            raise RuntimeError(result[0].error)
                        frame = world.tick()
                        image = interface.get_camera_images(frame_id=frame, timeout=60)
                        actual = interface.ego_vehicle.get_transform()
                        if actual.location.distance(target.location) > .001:
                            raise ValueError("Replayed pose did not match recorded pose")
                        if j >= 5:
                            images.append(image)
                    xyz, rot = build_ego_history(poses, list(range(len(rows))), anchor)
                    data = adapter.prepare_model_input(np.stack(images, axis=1), xyz, rot,
                                                       int(rows[anchor]["time_s"] * 1e6))
                    data["camera_indices"] = torch.tensor(adapter.source_camera_indices)
                    context = dict(original["context"],
                        index=anchor, time_s=rows[anchor]["time_s"], closed_loop=False,
                        world_pose={k: rows[anchor][k] for k in ("x", "y", "z", "yaw", "pitch", "roll")},
                        snapshot_frame=int(rows[anchor]["frame"]), render_frame=frame,
                        measured_speed_mps=rows[anchor]["speed"], measured_signed_speed_mps=rows[anchor]["speed"],
                        capture_mode="saved pose replay; no driving or model inference")
                    name = f"{original['name']}_anchor{position}"
                    destination = output / "inputs" / name
                    destination.mkdir(parents=True)
                    saved_path = destination / "input.pt"
                    torch.save(dict(data=data, context=context, instruction=instructions[anchor]), saved_path)
                    reference = base.recorded_reference(rows, context, np.arange(1, 31) * .1)
                    np.save(destination / "recorded_reference_world.npy", reference)
                    base.write_json(destination / "route.json", route)
                    case = dict(name=name, scene=original["name"], source=str(saved_path.resolve()),
                                input_sha256=base.sha256(saved_path), mission=original["mission"],
                                context=context, instruction=instructions[anchor],
                                source_rollout=str(folder), recorded_path_sha256=base.sha256(folder / "000_verifai/ego.csv"),
                                reference_sha256=base.sha256(destination / "recorded_reference_world.npy"),
                                route_sha256=base.sha256(destination / "route.json"))
                    cases.append(case)
                    base.write_json(output / "capture-progress.json", cases)
                    print(json.dumps(dict(captured=name, time_s=context["time_s"], speed=rows[anchor]["speed"])), flush=True)
            finally:
                for sensor in interface.sensors.values():
                    sensor.stop()
                    sensor.destroy()
                if interface.ego_vehicle is not None:
                    interface.ego_vehicle.destroy()
                world.tick()
    finally:
        for light, state, frozen in lights:
            light.set_state(state)
            light.freeze(frozen)
        world.set_weather(original_weather)
        world.apply_settings(original_settings)
    paths = list(old["source_sha256"]) + [str(Path(__file__).relative_to(ROOT)),
                "module/carla_interface.py", "module/open_loop_dataset.py", "local/experiments/projected_probe.py"]
    (output / "sources").mkdir()
    sources = {}
    for name in paths:
        sources[name] = base.sha256(ROOT / name)
        target = output / "sources" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    base.write_json(output / "protocol.json", dict(cases=cases, seeds=[42, 43], horizons_s=[1, 3],
        variants=list(VARIANTS), diffusion_seed_offset=10000, samples_per_inference=1,
        attention="sdpa", text_temperature=.6, top_p=.98, flow_steps=10, max_text_tokens=256,
        prediction_horizon_s=6.4, anchor_rule="Three equally spaced eligible row indices from 1.5s to duration minus 3.1s; fixed before inference.",
        activation_quantization="A8/A4: FP32 symmetric per-token absmax QDQ on last dimension at every NF4 linear input; BF16 compute. No calibration or native integer activation execution.",
        reference_limit=old["reference_limit"], scope=old["scope"],
        population_limit="30 correlated snapshots from 10 empty-scene assisted rollouts, 2 repeated seeds per snapshot. Bootstrap clusters by scene.",
        timing_limit="Shared GPUs; BF16 may use CPU weight offload. New timing is diagnostic, not a precision speed comparison.",
        source_sha256=sources))


def run(output, variant, cpu_offload=False, limit=None, verify_old=False, device_map_path=None):
    from local.experiments.inference_probe import Probe, acquire_experiment_lock
    from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
    from module.alpamayo_compat import patch_legacy_hydra_targets
    from alpamayo1_5.models.alpamayo1_5 import Alpamayo1_5
    from alpamayo1_5 import helper

    if torch.cuda.device_count() != 1 and device_map_path is None:
        raise ValueError("Multiple GPUs require an explicit coordinated device map")
    protocol = json.loads((output / "protocol.json").read_text())
    for name, digest in protocol["source_sha256"].items():
        if base.sha256(ROOT / name) != digest:
            raise ValueError(f"Source changed after freeze: {name}")
    with acquire_experiment_lock():
        destination = output / variant
        destination.mkdir(exist_ok=False)
        patch_legacy_hydra_targets()
        checkpoint = (ROOT.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / base.SNAPSHOT
                      if variant == "bf16" else OLD / "nf4/checkpoint")
        device_map = {"": 0}
        if device_map_path is not None:
            if cpu_offload or variant != "bf16":
                raise ValueError("Explicit sharding is only used for resident BF16")
            device_map = json.loads(Path(device_map_path).read_text())
            for i in range(36):
                if device_map[f"vlm.model.language_model.layers.{i}"] != device_map[f"expert.layers.{i}"]:
                    raise ValueError("VLM KV cache and matching expert layer must share a GPU")
        if cpu_offload:
            if variant != "bf16":
                raise ValueError("Only BF16 uses CPU offload")
            device_map = {"vlm.model.visual": 0, "vlm.model.language_model.embed_tokens": 0,
                          "vlm.model.language_model.layers": "cpu", "vlm.lm_head": "cpu",
                          "vlm.model.language_model.norm": 0, "vlm.model.language_model.rotary_emb": 0,
                          "expert": 0, "action_space": 0, "diffusion": 0, "action_in_proj": 0, "action_out_proj": 0}
        model = Alpamayo1_5.from_pretrained(str(checkpoint), dtype=torch.bfloat16,
                    device_map=device_map, attn_implementation="sdpa").eval()
        adapter = Alpamayo15Adapter()
        adapter.probe_processor = helper.get_processor(model.tokenizer)
        # Loaded exports store quantization_config as a plain dict in some versions.
        if isinstance(getattr(model.config, "quantization_config", None), dict):
            from transformers import BitsAndBytesConfig
            model.config.quantization_config = BitsAndBytesConfig.from_dict(model.config.quantization_config)
        report = base.model_report(model, "bf16" if variant == "bf16" else "nf4")
        handles, hooked = (install_activation_hooks(model, int(variant[-1]))
                           if variant in ("nf4_a8", "nf4_a4") else ([], []))
        report.update(variant=variant, activation_hook_layers=hooked, device_map=device_map,
                      cpu_offload=cpu_offload or "cpu" in device_map.values(), gpu=torch.cuda.get_device_name(),
                      resident_allocated_gib=sum(torch.cuda.memory_allocated(i) for i in range(torch.cuda.device_count())) / 2**30,
                      resident_allocated_gib_per_device=[torch.cuda.memory_allocated(i) / 2**30 for i in range(torch.cuda.device_count())],
                      peak_memory_note="Sum of per-device allocated peaks; these peaks need not occur simultaneously.",
                      timing_limit=protocol["timing_limit"])
        base.write_json(destination / "precision.json", report)
        print(json.dumps(dict(loaded=variant, resident_gib=report["resident_allocated_gib"], cpu_offload=cpu_offload)), flush=True)
        probe = Probe(model, destination)
        records = []
        try:
            if verify_old:
                old_case = json.loads((OLD / "protocol.json").read_text())["cases"][0]
                saved = torch.load(old_case["source"], map_location="cpu", weights_only=True)
                probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                               dict(seed=42, flow_seed=10042), "offload_validation")
                actual = json.loads((destination / "offload_validation.json").read_text())
                expected = json.loads((OLD / "bf16" / (old_case["name"] + "_seed42.json")).read_text())
                error = float(np.max(np.abs(np.array(actual["trajectory_ego"]) - np.array(expected["trajectory_ego"]))))
                verification = dict(max_coordinate_error_m=error,
                    identical_tokens=actual["generated_token_ids"] == expected["generated_token_ids"])
                base.write_json(destination / "offload_verification.json", verification)
                print(json.dumps(verification), flush=True)
                if error > 1e-5 or not verification["identical_tokens"]:
                    raise ValueError("Offloaded BF16 failed equivalence check")
            for warmup in range(2):
                saved = torch.load(protocol["cases"][0]["source"], map_location="cpu", weights_only=True)
                probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                               dict(seed=999, flow_seed=10999), f"warmup_{warmup}")
            for case in protocol["cases"][:limit]:
                if base.sha256(case["source"]) != case["input_sha256"]:
                    raise ValueError("Input changed after capture")
                saved = torch.load(case["source"], map_location="cpu", weights_only=True)
                folder = Path(case["source"]).parent
                for filename, key in (("route.json", "route_sha256"), ("recorded_reference_world.npy", "reference_sha256")):
                    if base.sha256(folder / filename) != case[key]:
                        raise ValueError("Scoring reference changed")
                route = json.loads((folder / "route.json").read_text())
                reference = np.load(folder / "recorded_reference_world.npy")
                for seed in protocol["seeds"]:
                    label = f"{case['name']}_seed{seed}"
                    record = dict(case=case["name"], scene=case["scene"], seed=seed, valid=False,
                                  input_sha256=case["input_sha256"])
                    try:
                        probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                            dict(seed=seed, flow_seed=10000 + seed), label)
                        raw = json.loads((destination / (label + ".json")).read_text())
                        trajectory, yaw = np.array(raw["trajectory_ego"]), np.array(raw["candidate_yaw_rad"])[0]
                        scores, world = score_plan(trajectory, yaw, saved["context"], route, reference, case["mission"])
                        record.update(scores, valid=bool(raw["trajectory_marker_present"]), finite=True,
                                      trajectory_marker_present=raw["trajectory_marker_present"],
                                      inference_seconds=raw["inference_seconds"],
                                      peak_allocated_gib=sum(raw["peak_allocated_gib"]),
                                      peak_allocated_gib_per_device=raw["peak_allocated_gib"],
                                      generated_tokens=raw["generated_tokens"],
                                      diffusion_rng=raw["diffusion_rng_fingerprint_used"])
                        np.savez(destination / (label + ".npz"), trajectory_ego=trajectory, yaw=yaw, world=world)
                    except (ValueError, FloatingPointError) as error:
                        record["error"] = str(error)
                    records.append(record)
                    base.write_json(destination / "scores.json", records)
                    print(json.dumps(dict(variant=variant, scored=label, valid=record["valid"],
                        route_rmse_3s=record.get("route_rmse_m_3s"), error=record.get("error"))), flush=True)
        finally:
            probe.close()
            for handle in handles:
                handle.remove()
        base.write_json(destination / "complete.json", dict(records=len(records)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("capture", "run"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--variant", choices=VARIANTS)
    parser.add_argument("--cpu-offload", action="store_true")
    parser.add_argument("--verify-old", action="store_true")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--device-map", type=Path)
    args = parser.parse_args()
    if args.stage == "capture":
        capture(args.output.resolve())
    else:
        run(args.output.resolve(), args.variant, args.cpu_offload, args.limit, args.verify_old, args.device_map)


if __name__ == "__main__":
    main()

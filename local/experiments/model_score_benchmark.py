"""Fixed CARLA-input model scores, without executing a downstream controller.

Run prepare, then run --precision bf16 / nf4 in separate processes, then summarize.
The recorded driven path is a controller-assisted BF16 reference, NOT expert GT.
Only one initial observation per scene is used; seeds are repeated measurements.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import time

import carla
import numpy as np
import torch

from local.experiments.audit_turn import project
from local.experiments.validated_selection import pose_transform, predicted_corners
from module import config
from module.trajectory_follower import trajectory_to_world

ROOT = Path(__file__).resolve().parents[2]
SNAPSHOT = "7aba8293c09993f2e125c6819df05d7fa3e873ea"
HORIZONS = (1, 3, 6)


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def recorded_reference(rows, context, times):
    """Interpolate logged rear-axle positions at exact future times; no extrapolation."""
    indices = [i for i, row in enumerate(rows) if int(row["frame"]) == context["snapshot_frame"]]
    if len(indices) != 1:
        raise ValueError("Capture frame must match exactly one recorded pose")
    anchor = rows[indices[0]]
    if np.linalg.norm([anchor[k] - context["world_pose"][k] for k in ("x", "y", "z")]) > .001:
        raise ValueError("Recorded pose disagrees with input anchor")
    recorded_times = np.array([row["time_s"] for row in rows]) - anchor["time_s"]
    if np.any(np.diff(recorded_times) <= 0):
        raise ValueError("Recorded timestamps must increase strictly")
    if times[0] < recorded_times[0] or times[-1] > recorded_times[-1] + 1e-6:
        raise ValueError("Reference must cover the whole requested horizon")
    positions = []
    for row in rows:
        rear = pose_transform(row).transform(carla.Location(x=config.EGO_REAR_AXLE_X))
        positions.append([rear.x, rear.y, rear.z])
    positions = np.array(positions)
    return np.stack([np.interp(times, recorded_times, positions[:, j]) for j in range(3)], axis=1)


def trajectory_scores(trajectory, yaw, context, route, reference, mission=None):
    trajectory, yaw = np.asarray(trajectory), np.asarray(yaw)
    if trajectory.shape != (64, 3) or yaw.shape != (64,):
        raise ValueError("Expected 64 raw trajectory waypoints and orientations")
    if not np.isfinite(trajectory).all() or not np.isfinite(yaw).all():
        raise ValueError("Nonfinite trajectory")
    transform = pose_transform(context["world_pose"])
    world = trajectory_to_world(transform, trajectory)
    corners = predicted_corners(transform, trajectory, yaw, context["vehicle_bounds"])
    origin = trajectory_to_world(transform, np.zeros((1, 3)))[0]
    distance, arc, _, _, _ = project(world[:, :2], route)
    _, initial_arc, _, _, _ = project(origin[None, :2], route)
    corner_distance, _, _, corner_width, _ = project(corners[:, :, :2].reshape(-1, 2), route)
    corner_margin = (corner_width - corner_distance).reshape(64, -1).min(axis=1)
    # Derivatives of raw positions; descriptive smoothness, not a safety threshold.
    velocity = np.diff(np.vstack([np.zeros((1, 2)), trajectory[:, :2]]), axis=0) / .1
    acceleration = np.diff(velocity, axis=0) / .1
    jerk = np.diff(acceleration, axis=0) / .1
    metrics = {}
    for horizon in HORIZONS:
        n = horizon * 10
        error = np.linalg.norm(world[:n, :2] - reference[:n, :2], axis=1)
        suffix = f"{horizon}s"
        metrics.update({
            f"recorded_reference_ade_m_{suffix}": float(error.mean()),
            f"recorded_reference_fde_m_{suffix}": float(error[-1]),
            f"route_rmse_m_{suffix}": float(np.sqrt(np.mean(distance[:n]**2))),
            f"corner_outside_fraction_{suffix}": float(np.mean(corner_margin[:n] < 0)),
            f"any_corner_outside_{suffix}": bool(np.any(corner_margin[:n] < 0)),
            f"minimum_corner_margin_m_{suffix}": float(corner_margin[:n].min()),
            f"route_progress_m_{suffix}": float(arc[n - 1] - initial_arc[0]),
            f"acceleration_rms_mps2_{suffix}": float(np.sqrt(np.mean(np.sum(acceleration[:n-1]**2, axis=1)))),
            f"jerk_rms_mps3_{suffix}": float(np.sqrt(np.mean(np.sum(jerk[:n-2]**2, axis=1)))),
        })
        if mission is not None:
            line = mission["stop_line"]
            angle = np.radians(line["yaw"])
            direction = np.array([np.cos(angle), np.sin(angle)])
            gaps = -np.max((corners[:n, :, :2] - [line["x"], line["y"]]) @ direction, axis=1)
            metrics[f"red_stop_line_crossed_{suffix}"] = bool(np.any(gaps < 0))
            metrics[f"red_minimum_front_gap_m_{suffix}"] = float(gaps.min())
    return metrics, world


def prepare(output):
    output.mkdir(parents=True, exist_ok=False)
    (output / "sources").mkdir()
    source_paths = [Path(__file__), ROOT / "local/experiments/inference_probe.py",
                    ROOT / "local/experiments/audit_turn.py", ROOT / "local/experiments/validated_selection.py",
                    ROOT / "module/trajectory_follower.py", ROOT / "module/adapters/alpamayo_1_5.py",
                    ROOT / "module/vlm_generate_optimization.py", ROOT / "module/config.py"]
    sources = {}
    for path in source_paths:
        target = output / "sources" / path.name
        shutil.copyfile(path, target)
        sources[str(path.relative_to(ROOT))] = sha256(path)
    manifest = json.loads((ROOT / "local/experiments/presets/ten-driving-scenes.json").read_text())
    cases = []
    for i, case in enumerate(manifest["scenarios"], 1):
        folder = "driving-investigation-19-ten-gain140" if i == 3 else "driving-investigation-19-stop-hold"
        capture = ROOT / "outputs" / folder / (case["name"] + "-route")
        source = capture / "inputs/proposal_000_00.pt"
        saved = torch.load(source, map_location="cpu", weights_only=True)
        if not torch.allclose(saved["data"]["ego_history_xyz"][0, 0, -1], torch.zeros(3), atol=1e-4):
            raise ValueError("Input history must end at the local-frame origin")
        if saved["context"]["index"] != 0:
            raise ValueError("This protocol uses only initial captures")
        csv_path = capture / "000_verifai/ego.csv"
        with csv_path.open() as stream:
            rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
        reference = recorded_reference(rows, saved["context"], np.arange(1, 61) * .1)
        route_path = capture / "route-reference.json"
        route = json.loads(route_path.read_text())["routes"][case["route"]]
        case_output = output / "inputs" / case["name"]
        case_output.mkdir(parents=True)
        np.save(case_output / "recorded_reference_world.npy", reference)
        write_json(case_output / "route.json", route)
        cases.append(dict(name=case["name"], source=str(source), input_sha256=sha256(source),
                          route_sha256=sha256(route_path), recorded_path_sha256=sha256(csv_path),
                          context=saved["context"], instruction=saved["instruction"],
                          mission=case.get("mission")))
    protocol = dict(checkpoint_snapshot=SNAPSHOT, cases=cases, seeds=list(range(42, 48)),
                    diffusion_seed_offset=10000, samples_per_inference=1, horizons_s=list(HORIZONS),
                    primary_horizon_s=3, prediction_horizon_s=6.4, warmup_calls=2,
                    text_temperature=.6, top_p=.98, flow_steps=10, attention="sdpa",
                    source_sha256=sources,
                    scope="Raw model outputs. No controller, map projection, candidate selection, VQA stop hint, format guard or plan reuse is applied.",
                    reference_limit="Recorded path is historical controller-assisted BF16 behavior, not independent expert ground truth. Route/corner metrics use the commanded lane corridor, not a full drivable-area or collision map.",
                    population_limit="Ten selected initial observations from empty CARLA scenes. Six seeds per scene are correlated repeats, not sixty independent scenarios.")
    write_json(output / "protocol.json", protocol)
    print(json.dumps(dict(prepared=str(output), cases=len(cases), runs_per_precision=len(cases)*6)), flush=True)


def model_report(model, precision):
    import bitsandbytes as bnb
    quantized, excluded = [], []
    for name, module in model.named_modules():
        if isinstance(module, bnb.nn.Linear4bit):
            state = module.weight.quant_state
            if state is None or module.weight.dtype != torch.uint8:
                raise RuntimeError(f"Layer {name} is not actually packed 4-bit")
            quantized.append(dict(name=name, original_shape=list(state.shape),
                                  stored_shape=list(module.weight.shape), storage_dtype=str(module.weight.dtype),
                                  quant_type=state.quant_type, blocksize=state.blocksize, nested=state.nested))
        elif isinstance(module, torch.nn.Linear):
            excluded.append(name)
    if precision == "nf4" and not quantized:
        raise RuntimeError("Requested NF4 but no quantized layers found")
    dtypes = Counter()
    for parameter in model.parameters():
        dtypes[str(parameter.dtype)] += parameter.numel() * parameter.element_size()
    return dict(precision=precision, quantized_layers=quantized, excluded_linear_layers=excluded,
                quantized_original_weight_elements=sum(int(np.prod(x["original_shape"])) for x in quantized),
                parameter_storage_bytes_by_dtype=dict(dtypes),
                parameter_storage_gib=sum(dtypes.values()) / 2**30,
                parameter_storage_note="Parameter tensors only; excludes quantization metadata, buffers, activations and allocator reservation. See measured CUDA memory for total runtime allocation.",
                quantization_config=model.config.quantization_config.to_dict() if hasattr(model.config, "quantization_config") else None)


def run(output, precision, export_checkpoint=False, case_limit=None, seed_limit=None):
    from local.experiments.inference_probe import Probe, acquire_experiment_lock
    from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
    from module.alpamayo_compat import patch_legacy_hydra_targets
    from alpamayo1_5.models.alpamayo1_5 import Alpamayo1_5
    from alpamayo1_5 import helper
    from transformers import BitsAndBytesConfig

    if torch.cuda.device_count() != 1:
        raise ValueError("Select exactly one idle GPU using CARLAMAYO_MODEL_GPUS")
    with acquire_experiment_lock():
        protocol = json.loads((output / "protocol.json").read_text())
        for name, digest in protocol["source_sha256"].items():
            if sha256(ROOT / name) != digest:
                raise ValueError(f"Source changed after protocol freeze: {name}")
        destination = output / precision
        destination.mkdir(exist_ok=False)
        patch_legacy_hydra_targets()
        checkpoint = ROOT.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / SNAPSHOT
        extra = {}
        if precision == "nf4":
            extra["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_use_double_quant=True, bnb_4bit_quant_type="nf4",
                llm_int8_skip_modules=["lm_head"])
        torch.cuda.empty_cache()
        started = time.perf_counter()
        model = Alpamayo1_5.from_pretrained(str(checkpoint), dtype=torch.bfloat16,
                                          device_map={"": 0}, attn_implementation="sdpa", **extra).eval()
        torch.cuda.synchronize()
        load_seconds = time.perf_counter() - started
        processor = helper.get_processor(model.tokenizer)
        adapter = Alpamayo15Adapter()
        adapter.probe_processor = processor
        report = model_report(model, precision)
        report.update(model_load_seconds=load_seconds, checkpoint=str(checkpoint),
                      torch_version=torch.__version__, cuda_version=torch.version.cuda,
                      library_versions={name: importlib.metadata.version(name) for name in (
                          "transformers", "accelerate", "bitsandbytes")},
                      gpu_name=torch.cuda.get_device_name(0), gpu_properties=str(torch.cuda.get_device_properties(0)),
                      gpu_snapshot=subprocess.check_output(["nvidia-smi", "--query-gpu=index,uuid,memory.used,utilization.gpu", "--format=csv,noheader"], text=True),
                      resident_allocated_gib=torch.cuda.memory_allocated() / 2**30)
        write_json(destination / "precision.json", report)
        print(json.dumps(dict(precision=precision, quantized_layers=len(report["quantized_layers"]),
                              resident_gib=report["resident_allocated_gib"], load_seconds=load_seconds)), flush=True)
        cases = protocol["cases"][:case_limit]
        seeds = protocol["seeds"][:seed_limit]
        probe = Probe(model, destination)
        records = []
        try:
            for warmup in range(protocol["warmup_calls"]):
                saved = torch.load(cases[0]["source"], map_location="cpu", weights_only=True)
                probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                               dict(seed=999, flow_seed=10999), f"warmup_{warmup}")
            for case in cases:
                if sha256(case["source"]) != case["input_sha256"]:
                    raise RuntimeError("Saved input changed")
                saved = torch.load(case["source"], map_location="cpu", weights_only=True)
                reference = np.load(output / "inputs" / case["name"] / "recorded_reference_world.npy")
                route = json.loads((output / "inputs" / case["name"] / "route.json").read_text())
                for seed in seeds:
                    label = f"{case['name']}_seed{seed}"
                    torch.cuda.synchronize()
                    started = time.perf_counter()
                    probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                                   dict(seed=seed, flow_seed=seed + protocol["diffusion_seed_offset"]), label)
                    torch.cuda.synchronize()
                    elapsed = time.perf_counter() - started
                    raw = json.loads((destination / (label + ".json")).read_text())
                    trajectory = np.asarray(raw["trajectory_ego"])
                    yaw = np.asarray(raw["candidate_yaw_rad"])[0]
                    finite = bool(np.isfinite(trajectory).all() and np.isfinite(yaw).all())
                    scores, world = trajectory_scores(trajectory, yaw, saved["context"], route, reference, case["mission"])
                    record = dict(case=case["name"], seed=seed, finite=finite,
                                  trajectory_marker_present=raw["trajectory_marker_present"],
                                  valid=finite and raw["trajectory_marker_present"],
                                  inference_seconds=raw["inference_seconds"],
                                  total_call_with_logging_seconds=elapsed,
                                  peak_allocated_gib=max(raw["peak_allocated_gib"]),
                                  peak_reserved_gib=torch.cuda.max_memory_reserved() / 2**30,
                                  generated_tokens=raw["generated_tokens"],
                                  input_sha256=case["input_sha256"],
                                  diffusion_rng=raw["diffusion_rng_fingerprint_used"], **scores)
                    records.append(record)
                    np.savez(destination / (label + ".npz"), trajectory_ego=trajectory, yaw=yaw, world=world)
                    write_json(destination / "scores.json", records)
                    print(json.dumps(dict(scored=label, precision=precision, valid=record["valid"],
                                          route_rmse_3s=record["route_rmse_m_3s"],
                                          reference_ade_3s=record["recorded_reference_ade_m_3s"])), flush=True)
        finally:
            probe.close()
        if export_checkpoint and precision == "nf4":
            model.save_pretrained(destination / "checkpoint", safe_serialization=True, max_shard_size="4GB")
            model.tokenizer.save_pretrained(destination / "checkpoint")
        write_json(destination / "complete.json", dict(records=len(records), cases=len(cases), seeds=seeds))


def aggregate(records):
    metrics = [k for k in records[0] if k.endswith(tuple(f"_{h}s" for h in HORIZONS))]
    result = {key: float(np.mean([row[key] for row in records if key in row])) for key in metrics}
    red_keys = sorted({key for row in records for key in row if key.startswith("red_")})
    for key in red_keys:
        result[key] = float(np.mean([row[key] for row in records if key in row]))
    moving = [row for row in records if not row["case"].startswith("10-")]
    for horizon in HORIZONS:
        result[f"moving_route_progress_m_{horizon}s"] = (float(np.mean([row[f"route_progress_m_{horizon}s"] for row in moving]))
                                                         if moving else None)
    result.update(valid_fraction=float(np.mean([r["valid"] for r in records])),
                  p50_inference_seconds=float(np.median([r["inference_seconds"] for r in records])),
                  p95_inference_seconds=float(np.percentile([r["inference_seconds"] for r in records], 95)),
                  peak_allocated_gib=max(r["peak_allocated_gib"] for r in records),
                  mean_generated_tokens=float(np.mean([r["generated_tokens"] for r in records])))
    return result


def summarize(output):
    protocol = json.loads((output / "protocol.json").read_text())
    rows = {p: json.loads((output / p / "scores.json").read_text()) for p in ("bf16", "nf4")}
    expected = {(case["name"], seed) for case in protocol["cases"] for seed in protocol["seeds"]}
    paired = []
    for precision in rows:
        if {(r["case"], r["seed"]) for r in rows[precision]} != expected or len(rows[precision]) != len(expected):
            raise RuntimeError(f"Incomplete or duplicate observations in {precision}")
    indexed = {p: {(r["case"], r["seed"]): r for r in records} for p, records in rows.items()}
    for case, seed in sorted(expected):
        a, b = indexed["bf16"][case, seed], indexed["nf4"][case, seed]
        if a["input_sha256"] != b["input_sha256"] or a["diffusion_rng"] != b["diffusion_rng"]:
            raise RuntimeError("Paired input or diffusion noise mismatch")
        label = f"{case}_seed{seed}"
        pred = {p: np.load(output / p / (label + ".npz"))["trajectory_ego"] for p in rows}
        displacement = np.linalg.norm(pred["nf4"][:, :2] - pred["bf16"][:, :2], axis=1)
        tokens = {p: json.loads((output / p / (label + ".json")).read_text())["generated_token_ids"] for p in rows}
        paired.append(dict(case=case, seed=seed, mean_prediction_change_m=float(displacement.mean()),
                           final_prediction_change_m=float(displacement[-1]), same_tokens=tokens["bf16"] == tokens["nf4"]))
    summaries = {p: aggregate(r) for p, r in rows.items()}
    by_case = {case["name"]: {p: aggregate([r for r in records if r["case"] == case["name"]])
                              for p, records in rows.items()} for case in protocol["cases"]}
    comparison = dict(summary=summaries,
                      nf4_minus_bf16={k: summaries["nf4"][k] - summaries["bf16"][k] for k in summaries["bf16"]},
                      per_case=by_case, paired_predictions=paired,
                      validity_note="Geometric scores include every finite output, including outputs missing the trajectory marker. The valid fraction is reported separately.",
                      reference_limit=protocol["reference_limit"], population_limit=protocol["population_limit"])
    write_json(output / "comparison.json", comparison)
    with (output / "summary.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "bf16", "nf4", "nf4_minus_bf16"])
        for key in summaries["bf16"]:
            writer.writerow([key, summaries["bf16"][key], summaries["nf4"][key], comparison["nf4_minus_bf16"][key]])
    with (output / "per_inference.csv").open("w") as stream:
        fields = ["precision", *sorted({k for records in rows.values() for row in records for k in row})]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for precision, records in rows.items():
            writer.writerows(dict(precision=precision, **record) for record in records)
    plot(output, protocol, rows, summaries)
    print(json.dumps(summaries, indent=2))


def plot(output, protocol, rows, summaries):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    titles = [("route_rmse_m_3s", "Raw-plan route RMSE at 3 s (m)"),
              ("recorded_reference_ade_m_3s", "Recorded-path ADE at 3 s (m)*")]
    for ax, (metric, title) in zip(axes[0], titles):
        for p, color, offset in (("bf16", "#355c9a", -.18), ("nf4", "#df7d32", .18)):
            values = [np.mean([r[metric] for r in rows[p] if r["case"] == c["name"]]) for c in protocol["cases"]]
            ax.bar(np.arange(10) + offset, values, .36, label=p.upper(), color=color)
        ax.set_xticks(np.arange(10), [str(i) for i in range(1, 11)])
        ax.set_xlabel("Scene number (six seeds per scene)")
        ax.set_title(title)
        ax.legend()
    for ax, metric, title in ((axes[1, 0], "p50_inference_seconds", "Median warmed inference (s)"),
                              (axes[1, 1], "peak_allocated_gib", "Peak allocated GPU memory (GiB)")):
        values = [summaries[p][metric] for p in ("bf16", "nf4")]
        bars = ax.bar(["BF16", "NF4"], values, color=["#355c9a", "#df7d32"])
        ax.bar_label(bars, fmt="%.2f")
        ax.set_ylim(0, max(values) * 1.15)
        ax.set_title(title)
    fig.suptitle("Alpamayo 1.5: raw predictions on ten fixed CARLA inputs")
    fig.text(.5, .015, "*Historical controller-assisted BF16 path; not independent expert ground truth.\nNo controller is executed in this test. 60 predictions per precision.", ha="center", fontsize=10)
    fig.tight_layout(rect=[0, .065, 1, .96])
    fig.savefig(output / "comparison.png", dpi=170)
    fig.savefig(output / "comparison.pdf")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("prepare", "run", "summarize"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--precision", choices=("bf16", "nf4"))
    parser.add_argument("--export-checkpoint", action="store_true")
    parser.add_argument("--case-limit", type=int)
    parser.add_argument("--seed-limit", type=int)
    args = parser.parse_args()
    if args.stage == "prepare":
        prepare(args.output)
    elif args.stage == "run":
        if args.precision is None:
            parser.error("run requires --precision")
        run(args.output, args.precision, args.export_checkpoint, args.case_limit, args.seed_limit)
    else:
        summarize(args.output)


if __name__ == "__main__":
    main()

"""Load the verified NF4 export in the unchanged projected-driving controller.

prepare creates matched T-junction/red-stop jobs only; it does not run CARLA.
capture uses the existing capture/audit pipeline with an explicit precision report.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys

from local.experiments.model_score_benchmark import ROOT, SNAPSHOT, sha256, write_json, model_report
from local.experiments.quant_explore import OLD


def validate_controller(options):
    required = dict(steering_mode="route", num_samples=1, steering_gain=1.4,
        controller_speed_cap_mps=4.5, speed_selection_attempts=6,
        speed_preference_mps=[3., 5.], projection_cache_age_s=3.,
        verify_signal=True, signal_check_interval=5, guard_trajectory_format=True,
        acknowledged_preroll=True, acknowledged_controls=True, wait_for_snapshot=True)
    for key, value in required.items():
        if options.get(key) != value:
            raise ValueError(f"Frozen controller mismatch: {key}")
    if bool(options.get("hold_visual_stop")) != (options["scene_policy"] == "red_light_stop"):
        raise ValueError("Visual hold must be enabled only in the red scene")
    if options.get("precision") not in ("bf16", "nf4"):
        raise ValueError("Expected BF16 or real NF4")


def verify_manifest(manifest_path, output, config, options, root=ROOT):
    """Enforce recorded sources and the selected job's exact config/options."""
    manifest = json.loads(Path(manifest_path).read_text())
    matching = [j for j in manifest["jobs"] if j["name"] == Path(output).name]
    if len(matching) != 1 or Path(output).parent.resolve() != Path(manifest_path).parent.resolve():
        raise ValueError("Output is not a unique job in this manifest")
    job = matching[0]
    for path, key in ((config, "config_sha256"), (options, "options_sha256")):
        if sha256(path) != job[key]:
            raise ValueError(f"Frozen job changed: {path}")
    for name, digest in manifest["source_sha256"].items():
        if sha256(root / name) != digest:
            raise ValueError(f"Frozen source changed: {name}")
    for name, digest in job.get("placement_sha256", {}).items():
        if sha256(Path(manifest_path).parent / name) != digest:
            raise ValueError(f"Frozen placement evidence changed: {name}")
    return job


def qualified_map(path, precision_path, verification_path):
    device_map = json.loads(Path(path).read_text())
    precision = json.loads(Path(precision_path).read_text())
    verification = json.loads(Path(verification_path).read_text())
    if device_map != precision["device_map"]:
        raise ValueError("Map differs from the qualified BF16 placement")
    if not verification["identical_tokens"] or verification["max_coordinate_error_m"] > 1e-5:
        raise ValueError("BF16 placement did not pass the existing equivalence gate")
    for i in range(36):
        if device_map[f"vlm.model.language_model.layers.{i}"] != device_map[f"expert.layers.{i}"]:
            raise ValueError("Paired VLM/expert layers must share a device")
    return device_map


def prepare(output, bf16_device_map=None):
    output.mkdir(parents=True, exist_ok=False)
    placement_hashes = {}
    if bf16_device_map is not None:
        qualification = ROOT / "outputs/quant-explore-20261107/shard-qualification/bf16"
        qualified_map(bf16_device_map, qualification / "precision.json", qualification / "offload_verification.json")
        for source, name in ((bf16_device_map, "bf16-device-map.json"),
                             (qualification / "precision.json", "bf16-qualified-precision.json"),
                             (qualification / "offload_verification.json", "bf16-placement-verification.json")):
            shutil.copyfile(source, output / name)
            placement_hashes[name] = sha256(output / name)
    old = json.loads((OLD / "protocol.json").read_text())
    jobs = []
    for case in old["cases"][-2:]:
        source = Path(case["source"]).parents[2]
        stem = case["name"] + "-route"
        original_options = json.loads((source / (stem + "-options.json")).read_text())
        for precision in ("bf16", "nf4"):
            name = case["name"] + "-" + precision
            config_path = output / (name + ".yaml")
            options_path = output / (name + "-options.json")
            shutil.copyfile(source / (stem + ".yaml"), config_path)
            options = dict(original_options, precision=precision)
            if precision == "bf16" and bf16_device_map is not None:
                options["model_placement"] = "coordinated_shards"
            validate_controller(options)
            write_json(options_path, options)
            checkpoint = (ROOT.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / SNAPSHOT
                          if precision == "bf16" else OLD / "nf4/checkpoint")
            command = [str(ROOT / ".venv/bin/python"), "-m", "local.experiments.nf4_closed_loop", "capture",
                       "--jobs-manifest", str(output / "jobs.json"),
                       "--config", str(config_path), "--options", str(options_path),
                       "--checkpoint", str(checkpoint), "--duration", "22" if case["name"].startswith("09-") else "12",
                       "--capture-frames", "0", "--output", str(output / name)]
            if precision == "bf16" and bf16_device_map is not None:
                command += ["--device-map", str(output / "bf16-device-map.json")]
            jobs.append(dict(name=name, precision=precision, command=command,
                config_sha256=sha256(config_path), options_sha256=sha256(options_path),
                placement_sha256=placement_hashes if precision == "bf16" else {},
                audit_command=[str(ROOT / ".venv/bin/python"), "-m", "local.experiments.audit_projection", str(output / name)]))
    sources = {str(p.relative_to(ROOT)): sha256(p) for folder in ("module", "local") for p in (ROOT/folder).rglob("*.py")}
    for name in sources:
        target = output / "sources" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    write_json(output / "jobs.json", dict(state="prepared_not_executed", jobs=jobs,
        comparison="Same controller, initial state, scenario seed and flow seed. Checkpoint precision and declared, numerically qualified model placement may differ; timing is not a precision comparison.",
        resource_requirement="Owned CARLA server on a separate render GPU; an available model GPU or validated CPU offload. Coordinate port2000 and device ownership first.",
        timing_limit="CPU-offloaded/shared-GPU runs establish quality outcomes only, not precision speed differences.",
        source_sha256=sources))


def capture(arguments, cpu_offload):
    import torch
    from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
    from local.experiments import fp8_quantization, projected_probe

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--options", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--jobs-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--device-map", type=Path)
    args, _ = parser.parse_known_args(arguments)
    job = verify_manifest(args.jobs_manifest, args.output, args.config, args.options)
    if arguments != job["command"][4:]:
        raise ValueError("Capture arguments differ from the frozen command, including duration/seed inputs")
    options = json.loads(args.options.read_text())
    validate_controller(options)
    precision = options["precision"]
    if precision != job["precision"]:
        raise ValueError("Precision does not match frozen job")
    device_map = None
    if args.device_map is not None:
        if precision != "bf16" or cpu_offload or options.get("model_placement") != "coordinated_shards":
            raise ValueError("Explicit sharding requires BF16 and its declared placement")
        device_map = qualified_map(args.device_map, args.jobs_manifest.parent / "bf16-qualified-precision.json",
                                   args.jobs_manifest.parent / "bf16-placement-verification.json")
    elif options.get("model_placement") != "single_gpu":
        raise ValueError("A coordinated placement requires its frozen device map")
    expected = (OLD / "nf4/checkpoint" if precision == "nf4" else
                ROOT.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / SNAPSHOT)
    if args.checkpoint.resolve() != expected.resolve():
        raise ValueError("Checkpoint must match the verified paired experiment")
    original_precision, original_loader = fp8_quantization.apply_precision, Alpamayo15Adapter.load_model
    runner = projected_probe.native.runner
    original_runner_loader = runner.load_sharded_model
    if device_map is not None:
        if torch.cuda.device_count() != 3:
            raise ValueError("The qualified map requires exactly three model GPUs")
        def coordinated_loader(runtime_args, adapter):
            model, processor = adapter.load_model(checkpoint_path=runtime_args.checkpoint,
                attn_implementation="sdpa", device_map=device_map)
            torch.cuda.set_device(0)
            model.eval()
            return model, processor
        runner.load_sharded_model = coordinated_loader
    if cpu_offload:
        verification = json.loads((ROOT / "outputs/quant-explore-20261107/bf16-cpu-offload-validation/offload_verification.json").read_text())
        if precision != "bf16" or not verification["identical_tokens"] or verification["max_coordinate_error_m"] > 1e-5:
            raise ValueError("CPU offload requires the matching BF16 equivalence validation")
        def offloaded_loader(self, *values, **kwargs):
            kwargs["device_map"] = {"vlm.model.visual": 0, "vlm.model.language_model.embed_tokens": 0,
                "vlm.model.language_model.layers": "cpu", "vlm.lm_head": "cpu",
                "vlm.model.language_model.norm": 0, "vlm.model.language_model.rotary_emb": 0,
                "expert": 0, "action_space": 0, "diffusion": 0, "action_in_proj": 0, "action_out_proj": 0}
            return original_loader(self, *values, **kwargs)
        Alpamayo15Adapter.load_model = offloaded_loader
    def report_precision(model, requested):
        if requested == "bf16":
            report = original_precision(model, requested)
        elif requested == "nf4":
            if isinstance(model.config.quantization_config, dict):
                from transformers import BitsAndBytesConfig
                model.config.quantization_config = BitsAndBytesConfig.from_dict(model.config.quantization_config)
            report = model_report(model, "nf4")
            if len(report["quantized_layers"]) != 624:
                raise ValueError("Export must contain the verified 624 packed NF4 layers")
            report.update(compressed_weight_storage=True, native_fp8_compute=False,
                          activation_quantization="none", compute_dtype="bfloat16")
        else:
            raise ValueError(requested)
        report.update(cpu_offload=cpu_offload or "cpu" in model.hf_device_map.values(), device_map=model.hf_device_map,
                      model_placement=options["model_placement"], input_device="cuda:0",
                      bridge_source_sha256=sha256(Path(__file__)), jobs_manifest_sha256=sha256(args.jobs_manifest))
        return report
    fp8_quantization.apply_precision = report_precision
    try:
        if device_map is not None:
            for i, fraction in enumerate((.40, .40, .16)):
                torch.cuda.set_per_process_memory_fraction(fraction, i)
        else:
            torch.cuda.set_per_process_memory_fraction(.41 if cpu_offload or precision == "nf4" else .98)
        torch.cuda.set_device(0)
        forwarded = list(arguments)
        index = forwarded.index("--jobs-manifest")
        del forwarded[index:index+2]
        if "--device-map" in forwarded:
            index = forwarded.index("--device-map")
            del forwarded[index:index+2]
        sys.argv = [str(Path(__file__)), "capture", *forwarded]
        projected_probe.main()
    finally:
        fp8_quantization.apply_precision, Alpamayo15Adapter.load_model = original_precision, original_loader
        runner.load_sharded_model = original_runner_loader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("prepare", "capture"))
    parser.add_argument("--cpu-offload", action="store_true")
    args, rest = parser.parse_known_args()
    if args.stage == "prepare":
        sub = argparse.ArgumentParser()
        sub.add_argument("--output", type=Path, required=True)
        sub.add_argument("--bf16-device-map", type=Path)
        prepared = sub.parse_args(rest)
        prepare(prepared.output.resolve(), prepared.bf16_device_map)
    else:
        capture(rest, args.cpu_offload)


if __name__ == "__main__":
    main()

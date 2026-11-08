"""Paired BF16/FP8 inference on preserved first inputs from the ten-scene suite."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from local.experiments import inference_probe as native
from local.experiments.fp8_quantization import PRECISIONS, apply_precision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cases", type=int, nargs="+", default=list(range(1, 11)))
    parser.add_argument("--precision", choices=PRECISIONS[1:], default=PRECISIONS[1])
    args = parser.parse_args()
    if torch.cuda.device_count() != 1:
        raise ValueError("Select one model GPU with CARLAMAYO_MODEL_GPUS")
    with native.acquire_experiment_lock():
        args.output.mkdir(parents=True, exist_ok=False)
        root = native.runner.ROOT
        manifest = json.loads((root / "local/experiments/presets/ten-driving-scenes.json").read_text())
        inputs = []
        for number in args.cases:
            case = manifest["scenarios"][number - 1]
            folder = "driving-investigation-19-ten-gain140" if number == 3 else "driving-investigation-19-stop-hold"
            capture = root / "outputs" / folder / (case["name"] + "-route")
            source = sorted((capture / "inputs").glob("*.pt"))[0]
            original = json.loads((capture / (source.stem + ".json")).read_text())
            inputs.append((case["name"], source, original["options"]))
        native.write_json(args.output / "inputs.json", [dict(name=name, path=str(source),
            sha256=hashlib.sha256(source.read_bytes()).hexdigest(), options=options)
            for name, source, options in inputs])
        adapter = native.Alpamayo15Adapter()
        checkpoint = root.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / native.runner.SNAPSHOT
        model, processor = adapter.load_model(checkpoint_path=str(checkpoint),
            attn_implementation="sdpa", device_map={"": 0}, max_memory={0: "23GiB"})
        model.eval()
        adapter.probe_processor = processor
        probe = native.Probe(model, args.output)
        try:
            for precision in ("bf16", args.precision):
                native.write_json(args.output / (precision + "-precision.json"), apply_precision(model, precision))
                for name, source, options in inputs:
                    saved = torch.load(source, map_location="cpu", weights_only=True)
                    probe.previous_plan = None
                    probe.visual_signal_index = -1000
                    probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                                   {**options, "precision": precision}, name + "-" + precision)
        finally:
            probe.close()
        results = []
        for name, _, _ in inputs:
            a = json.loads((args.output / (name + "-bf16.json")).read_text())
            b = json.loads((args.output / (name + "-" + args.precision + ".json")).read_text())
            errors = np.linalg.norm(np.array(a["trajectory_ego"]) - np.array(b["trajectory_ego"]), axis=-1)
            if not np.isfinite(errors).all():
                raise RuntimeError(f"Non-finite trajectory for {name}")
            results.append(dict(name=name, trajectory_mean_displacement_m=float(errors.mean()),
                trajectory_final_displacement_m=float(errors[-1]),
                bf16_preview_mps=a["controller_preview_speed_mps"], fp8_preview_mps=b["controller_preview_speed_mps"],
                bf16_seconds=a["inference_seconds"], fp8_seconds=b["inference_seconds"],
                bf16_marker=a["trajectory_marker_present"], fp8_marker=b["trajectory_marker_present"]))
        native.write_json(args.output / "comparison.json", results)
        print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()

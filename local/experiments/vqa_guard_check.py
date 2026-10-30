"""Compare an archived and current guard on identical saved VQA inputs."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import time

import torch

from local import run_intersection as runner
from local.experiments import inference_probe as current
from local.experiments.camera_projection import project_history
from module.adapters.alpamayo_1_5 import Alpamayo15Adapter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, nargs="+", required=True)
    parser.add_argument("--legacy-probe", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint")
    args = parser.parse_args()
    with current.acquire_experiment_lock():
        args.output.mkdir(parents=True, exist_ok=False)
        spec = importlib.util.spec_from_file_location("archived_inference_probe", args.legacy_probe)
        legacy = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(legacy)
        current.write_json(args.output / "sources.json", {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (args.legacy_probe, Path(current.__file__), Path(__file__))})
        adapter = Alpamayo15Adapter()
        model, processor = runner.load_sharded_model(args, adapter)
        question = "What color is the traffic light facing the ego vehicle? Answer only red, yellow, green, or unknown."
        results = []
        for path in args.inputs:
            saved = torch.load(path, map_location="cpu", weights_only=True)
            data = saved["data"]
            if saved["context"].get("render_projection") == "wide_pinhole":
                data = project_history(data, "equidistant")
            for name, cls, guard, cameras in (
                ("legacy_guarded", legacy.Probe, True, None),
                ("fixed_guarded", current.Probe, True, None),
                ("unguarded", current.Probe, False, None),
                ("fixed_front", current.Probe, True, [1]),
                ("fixed_front_tele", current.Probe, True, [1, 6]),
            ):
                subset = data
                if cameras:
                    indices = data["camera_indices"].tolist()
                    selected = [indices.index(i) for i in cameras]
                    subset = {**data, "image_frames": data["image_frames"][selected],
                              "camera_indices": data["camera_indices"][selected]}
                probe = cls(model, args.output)
                probe.options = {"guard_trajectory_format": guard}
                torch.manual_seed(42)
                torch.cuda.manual_seed_all(42)
                started = time.monotonic()
                try:
                    response = adapter.run_vqa(model, processor, subset, question)
                    answer = adapter.extract_answer_text(response).strip()
                    result = {"input": str(path), "variant": name, "answer": answer,
                              "colors": sorted(set(re.findall(r"\b(red|yellow|green)\b", answer.lower()))),
                              "seconds": time.monotonic() - started,
                              "tokens": probe.trace["generated_token_ids"],
                              "raw_generated_text": probe.trace["raw_generated_text"]}
                    results.append(result)
                    current.write_json(args.output / "results.json", results)
                    print(json.dumps({"input": path.name, "variant": name, "answer": answer,
                                      "tokens": len(result["tokens"]), "seconds": result["seconds"]}), flush=True)
                finally:
                    probe.close()


if __name__ == "__main__":
    main()

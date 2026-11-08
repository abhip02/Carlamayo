"""Independently recompute saved scores and optionally replay the exported NF4 model."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from local.experiments.model_score_benchmark import ROOT, sha256, trajectory_scores, write_json


def verify(output, reload_checkpoint=False):
    protocol = json.loads((output / "protocol.json").read_text())
    expected_count = len(protocol["cases"]) * len(protocol["seeds"])
    for name, digest in protocol["source_sha256"].items():
        if sha256(ROOT / name) != digest:
            raise RuntimeError(f"Benchmark source changed: {name}")
    checked = 0
    for precision in ("bf16", "nf4"):
        rows = json.loads((output / precision / "scores.json").read_text())
        if len(rows) != expected_count:
            raise RuntimeError("Missing inference results")
        indexed = {(row["case"], row["seed"]): row for row in rows}
        for case in protocol["cases"]:
            if sha256(case["source"]) != case["input_sha256"]:
                raise RuntimeError("Input changed")
            route = json.loads((output / "inputs" / case["name"] / "route.json").read_text())
            reference = np.load(output / "inputs" / case["name"] / "recorded_reference_world.npy")
            for seed in protocol["seeds"]:
                label = f"{case['name']}_seed{seed}"
                raw = json.loads((output / precision / (label + ".json")).read_text())
                if raw["options"] != dict(seed=seed, flow_seed=seed+protocol["diffusion_seed_offset"]):
                    raise RuntimeError("Unexpected inference intervention")
                if raw["decoder_estimated_speed_mps"] != raw["decoder_used_speed_mps"]:
                    raise RuntimeError("Decoder speed was modified")
                arrays = np.load(output / precision / (label + ".npz"))
                np.testing.assert_array_equal(arrays["trajectory_ego"], raw["trajectory_ego"])
                scores, world = trajectory_scores(arrays["trajectory_ego"], arrays["yaw"],
                                                   case["context"], route, reference, case["mission"])
                np.testing.assert_allclose(world, arrays["world"], rtol=0, atol=1e-9)
                for key, value in scores.items():
                    np.testing.assert_allclose(value, indexed[case["name"], seed][key], rtol=0, atol=1e-9)
                checked += 1
    report = dict(recomputed_predictions=checked, unchanged_sources=True, unchanged_inputs=True,
                  unmodified_raw_decoder=True)
    if reload_checkpoint:
        from local.experiments.inference_probe import Probe, acquire_experiment_lock
        from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
        from module.alpamayo_compat import patch_legacy_hydra_targets
        from alpamayo1_5.models.alpamayo1_5 import Alpamayo1_5
        from alpamayo1_5 import helper
        import bitsandbytes as bnb
        if torch.cuda.device_count() != 1:
            raise ValueError("Choose one idle model GPU")
        with acquire_experiment_lock():
            patch_legacy_hydra_targets()
            checkpoint = output / "nf4/checkpoint"
            model = Alpamayo1_5.from_pretrained(str(checkpoint), dtype=torch.bfloat16,
                                              device_map={"": 0}, attn_implementation="sdpa").eval()
            packed = [m for m in model.modules() if isinstance(m, bnb.nn.Linear4bit)]
            if len(packed) != 624 or any(m.weight.dtype != torch.uint8 for m in packed):
                raise RuntimeError("Export did not retain packed NF4 weights")
            destination = output / "export-reload"
            destination.mkdir(exist_ok=False)
            adapter = Alpamayo15Adapter()
            adapter.probe_processor = helper.get_processor(model.tokenizer)
            case, seed = protocol["cases"][0], protocol["seeds"][0]
            saved = torch.load(case["source"], map_location="cpu", weights_only=True)
            probe = Probe(model, destination)
            try:
                probe.evaluate(adapter, saved["data"], saved["context"], saved["instruction"],
                               dict(seed=seed, flow_seed=seed+protocol["diffusion_seed_offset"]), "verification")
            finally:
                probe.close()
            replay = json.loads((destination / "verification.json").read_text())
            original = json.loads((output / "nf4" / f"{case['name']}_seed{seed}.json").read_text())
            difference = np.asarray(replay["trajectory_ego"]) - np.asarray(original["trajectory_ego"])
            np.testing.assert_allclose(replay["trajectory_ego"], original["trajectory_ego"], rtol=1e-5, atol=1e-5)
            if replay["generated_token_ids"] != original["generated_token_ids"]:
                raise RuntimeError("Exported model generated different reasoning tokens")
            report["export_reload"] = dict(packed_layers=len(packed), identical_tokens=True,
                                           maximum_absolute_trajectory_difference_m=float(np.max(np.abs(difference))),
                                           checkpoint_size_bytes=sum(p.stat().st_size for p in checkpoint.iterdir() if p.is_file()))
    write_json(output / "verification.json", report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--reload-checkpoint", action="store_true")
    args = parser.parse_args()
    verify(args.output, args.reload_checkpoint)

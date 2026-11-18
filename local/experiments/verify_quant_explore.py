"""Recompute maneuver scores from raw outputs and verify paired experiment inputs."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from local.experiments.quant_explore import ROOT, VARIANTS, score_plan
from local.experiments.model_score_benchmark import sha256, write_json
from local.experiments.audit_turn import project
from local.experiments.validated_selection import predicted_corners, pose_transform


def verify(output):
    protocol = json.loads((output / "protocol.json").read_text())
    for name, digest in protocol["source_sha256"].items():
        assert sha256(ROOT / name) == digest, name
    cases = {c["name"]: c for c in protocol["cases"]}
    for case in cases.values():
        folder = Path(case["source"]).parent
        assert sha256(case["source"]) == case["input_sha256"]
        assert sha256(folder / "recorded_reference_world.npy") == case["reference_sha256"]
        assert sha256(folder / "route.json") == case["route_sha256"]
        saved = torch.load(case["source"], weights_only=True, map_location="cpu")
        assert saved["data"]["image_frames"].shape == (4, 4, 3, 1080, 1920)
        assert saved["data"]["ego_history_xyz"].shape == (1, 1, 16, 3)
        assert torch.max(torch.abs(saved["data"]["ego_history_xyz"][0, 0, -1])).item() < 1e-4
        assert saved["context"] == case["context"]
        assert saved["instruction"] == case["instruction"]
    result = dict(input_count=len(cases), source_hashes_match=True, input_hashes_match=True,
                  expected_predictions_per_variant=len(cases)*len(protocol["seeds"]), variants={})
    for variant in VARIANTS:
        directory = output / variant
        complete = (directory / "complete.json").exists()
        if not (directory / "scores.json").exists():
            result["variants"][variant] = dict(complete=False, attempted=0,
                not_attempted=result["expected_predictions_per_variant"])
            continue
        records = json.loads((directory / "scores.json").read_text())
        expected = {(c, s) for c in cases for s in protocol["seeds"]}
        actual = {(r["case"], r["seed"]) for r in records}
        assert actual <= expected
        assert len(records) == len(actual)
        if complete:
            assert actual == expected
        maximum_error, verified, invalid = 0., 0, 0
        endpoint_cases = []
        for row in records:
            case = cases[row["case"]]
            folder = Path(case["source"]).parent
            label = f"{row['case']}_seed{row['seed']}"
            raw_path = directory / (label + ".json")
            raw = json.loads(raw_path.read_text()) if raw_path.exists() else None
            if raw is not None:
                assert raw["options"] == dict(seed=row["seed"], flow_seed=row["seed"]+10000)
                assert raw["instruction"] == case["instruction"]
                assert raw["context"] == case["context"]
                assert raw["decoder_estimated_speed_mps"] == raw["decoder_used_speed_mps"]
                assert raw["input_camera_indices"] == [0, 1, 2, 6]
            if not row["valid"]:
                if raw is not None:
                    trajectory = np.asarray(raw["trajectory_ego"], dtype=float)
                    yaw = np.asarray(raw["candidate_yaw_rad"], dtype=float)
                    eligible = trajectory.shape == (64, 3) and yaw.shape == (1, 64)
                    eligible = eligible and np.isfinite(trajectory).all() and np.isfinite(yaw).all()
                    assert not (eligible and raw["trajectory_marker_present"]), (label, "valid output incorrectly excluded")
                else:
                    assert row.get("error"), (label, "missing output without explicit error")
                invalid += 1
                continue
            assert raw is not None
            assert len(raw["candidate_yaw_rad"]) == 1
            if raw["candidate_trajectories"] is not None:
                assert len(raw["candidate_trajectories"]) == 1
            assert raw["input_camera_indices"] == [0, 1, 2, 6]
            trajectory, yaw = np.array(raw["trajectory_ego"]), np.array(raw["candidate_yaw_rad"])[0]
            route = json.loads((folder / "route.json").read_text())
            reference = np.load(folder / "recorded_reference_world.npy")
            scores, world = score_plan(trajectory, yaw, case["context"], route, reference, case["mission"])
            for key, value in scores.items():
                error = abs(float(value)-float(row[key]))
                maximum_error = max(maximum_error, error)
                assert error < 1e-9, (label, key, error)
            stored = np.load(directory / (label + ".npz"))
            np.testing.assert_array_equal(stored["trajectory_ego"], trajectory)
            np.testing.assert_allclose(stored["world"], world, atol=1e-7, rtol=0)
            corners = predicted_corners(pose_transform(case["context"]["world_pose"]), trajectory, yaw, case["context"]["vehicle_bounds"])
            _, arc, *_ = project(np.vstack([world[:30, :2], corners[:30, :, :2].reshape(-1,2)]), route)
            length = np.linalg.norm(np.diff([[p["x"], p["y"]] for p in route["points"]], axis=0), axis=1).sum()
            if np.any((arc < 1e-5) | (arc > length-1e-5)):
                endpoint_cases.append(label)
            verified += 1
        precision = json.loads((directory / "precision.json").read_text())
        if variant != "bf16":
            assert len(precision["quantized_layers"]) == 624
            assert all(x["storage_dtype"] == "torch.uint8" for x in precision["quantized_layers"])
        if variant in ("nf4_a8", "nf4_a4"):
            assert len(precision["activation_hook_layers"]) == 624
        result["variants"][variant] = dict(complete=complete, attempted=len(records),
            not_attempted=len(expected)-len(records), recomputed_valid_predictions=verified,
            invalid_predictions=invalid, maximum_score_recompute_error=maximum_error,
            unmodified_native_decoder=True, matched_inputs_and_seeds=True,
            plans_with_route_endpoint_projection_3s=endpoint_cases,
            endpoint_note="Rear axle or footprint corner projected to a finite route endpoint; a corridor departure there can conflate lateral error and insufficient route support.")
    write_json(output / "verification.json", result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    verify(parser.parse_args().output.resolve())

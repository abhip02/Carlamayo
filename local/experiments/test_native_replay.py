"""Rejected-proposal replays must still use natively valid, unchanged samples."""
import json

import numpy as np
import pytest

from local.experiments.replay_native_tracking import native_proposal
from local.experiments.test_retained_probe import setup_plan


def files(root):
    route, _, _, plan, _ = setup_plan()
    paths = [plan.record["trajectory_ego"], (np.asarray(plan.record["trajectory_ego"]) + [0, 3, 0]).tolist()]
    proposal = {"context": plan.record["context"], "candidate_trajectories": paths,
                "candidate_generated_tokens": [[1, 2, 3], [4, 5, 6]],
                "candidate_yaw_rad": [[0.] * 64] * 2,
                "candidate_velocities": [[3.] * 64] * 2,
                "candidate_markers": [True, True], "visual_signal": "green"}
    (root / "proposals").mkdir()
    (root / "proposals/pool.json").write_text(json.dumps(proposal))
    (root / "experiment.json").write_text(json.dumps({"options": {
        "validation_route": "straight", "validation_horizon_s": 1.5,
        "validation_body_margin_m": .2, "validation_green_min_target_mps": 1.}}))
    (root / "route-reference.json").write_text(json.dumps({"routes": {"straight": route}}))
    return proposal


def test_rejected_proposal_preserves_native_arrays_tokens_anchor_and_source_file(tmp_path):
    proposal = files(tmp_path)
    before = (tmp_path / "proposals/pool.json").read_bytes()
    record, world, speeds = native_proposal(tmp_path, "pool", 0)
    assert record["trajectory_ego"] == proposal["candidate_trajectories"][0]
    assert record["generated_token_ids"] == proposal["candidate_generated_tokens"][0]
    assert record["context"] == proposal["context"]
    assert record["native_admission"]["valid"]
    assert not world.flags.writeable and not speeds.flags.writeable
    assert (tmp_path / "proposals/pool.json").read_bytes() == before


@pytest.mark.parametrize("candidate,reason", [(1, "fails original native validation"), (2, "original candidate index")])
def test_replay_refuses_invalid_native_geometry_and_nonexistent_sample(tmp_path, candidate, reason):
    files(tmp_path)
    with pytest.raises(ValueError, match=reason):
        native_proposal(tmp_path, "pool", candidate)

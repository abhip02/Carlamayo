"""Real VerifAI sampling and launcher handoff; no CARLA or model required."""
from copy import deepcopy
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

pytest.importorskip("verifai")
import numpy as np
import yaml

from local import run_verifai


@pytest.fixture
def config():
    return yaml.safe_load((run_verifai.ROOT / "local/configs/verifai.yaml").read_text())


def test_samples_are_repeatable_bounded_and_independent_of_policy_seed(config):
    config["sampling"]["count"] = 25
    original = deepcopy(config)
    first = run_verifai.sample_config(config)
    assert config == original
    assert first == run_verifai.sample_config(config)
    config["seed"] += 1
    assert first["rollouts"] == run_verifai.sample_config(config)["rollouts"]
    config["sampling"]["seed"] += 1
    assert first["rollouts"] != run_verifai.sample_config(config)["rollouts"]
    assert len(first["rollouts"]) == 25
    for spec in first["rollouts"]:
        for field, (low, high) in config["sampling"]["ranges"].items():
            assert low <= spec[field] <= high
        assert spec["lateral_offset_m"] == 0.0


def test_limit_preserves_sample_prefix_and_records_actual_count(config):
    config["sampling"]["count"] = 5
    full = run_verifai.sample_config(config)
    limited = run_verifai.sample_config(config, limit=2)
    assert limited["rollouts"] == full["rollouts"][:2]
    assert limited["sampling_info"]["sample_count"] == 2


def test_sampling_preserves_callers_random_streams(config):
    # Warm imports before checking the RNG state owned by the caller.
    run_verifai.sample_config(config)
    py_state, np_state = random.getstate(), np.random.get_state()
    run_verifai.sample_config(config)
    assert random.getstate() == py_state
    after = np.random.get_state()
    assert after[0] == np_state[0]
    np.testing.assert_array_equal(after[1], np_state[1])
    assert after[2:] == np_state[2:]


@pytest.mark.parametrize("bounds", [[4, 3], [-1, 4], [0, float("nan")], [0], [True, 4], ["0", 4]])
def test_invalid_speed_ranges_are_rejected(config, bounds):
    config["sampling"]["ranges"]["speed_mps"] = bounds
    with pytest.raises(ValueError, match="speed_mps"):
        run_verifai.sample_config(config)


@pytest.mark.parametrize("key,value", [("count", 0), ("count", True), ("seed", -1), ("seed", 2**32)])
def test_invalid_sampling_settings_are_rejected(config, key, value):
    config["sampling"][key] = value
    with pytest.raises(ValueError, match=key):
        run_verifai.sample_config(config)


def test_existing_rollout_list_cannot_be_accidentally_resampled(config):
    resolved = run_verifai.sample_config(config)
    with pytest.raises(ValueError, match="instead of rollouts"):
        run_verifai.sample_config(resolved)


def test_sample_only_prints_replayable_yaml_without_launching(config, monkeypatch, capsys):
    def unexpected_launch(*args):
        pytest.fail("sample-only must not launch CARLA or the model")
    monkeypatch.setattr(run_verifai, "launch", unexpected_launch)
    assert run_verifai.main(["--sample-only", "--limit", "1", "--duration", "0.2"]) == 0
    resolved = yaml.safe_load(capsys.readouterr().out)
    assert resolved["rollouts"] == run_verifai.sample_config(config)["rollouts"][:1]
    assert resolved["duration_s"] == 0.2


def test_launcher_passes_all_samples_to_one_existing_batch_and_propagates_failure(config, monkeypatch):
    resolved = run_verifai.sample_config(config)
    calls = []

    def fake_run(command, cwd):
        calls.append(command)
        assert Path(command[1]) == run_verifai.ROOT / "local/run.sh"
        assert yaml.safe_load(Path(command[3]).read_text()) == resolved
        assert command[4:] == ["--host", "localhost", "--port", "2001"]
        assert cwd == run_verifai.ROOT
        return SimpleNamespace(returncode=7)

    monkeypatch.setattr(run_verifai.subprocess, "run", fake_run)
    assert run_verifai.launch(resolved, ["--host", "localhost", "--port", "2001"]) == 7
    assert len(calls) == 1
    assert not Path(calls[0][3]).exists()

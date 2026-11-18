"""Real Scenic constraints and VerifAI feedback without CARLA or model loading."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import random

import pytest

pytest.importorskip("verifai")
import numpy as np
from scenic.core.distributions import RejectionException
import yaml

from local import run_verifai, verifai_server
from local.scenic_sampling import IntersectionScenicSampler, parameters_to_spec
from local.verifai_client import run_client

EXAMPLE = run_verifai.ROOT / "local/scenarios/intersection.scenic"
PARAMETERS = """
param distance_before_intersection_m = 22
param lateral_offset_m = 0
param speed_mps = 3
param heading_offset_deg = 0
ego = new Object
"""


def sample_example(seed, count):
    random.seed(seed)
    np.random.seed(seed)
    sampler = IntersectionScenicSampler.fromScenario(str(EXAMPLE))
    return [sampler.rollout_spec(sampler.nextSample()) for _ in range(count)]


def test_example_is_repeatable_and_constraints_hold():
    first = sample_example(42, 30)
    assert first == sample_example(42, 30)
    assert first != sample_example(43, 30)
    for spec in first:
        assert 20 <= spec["distance_before_intersection_m"] <= 25
        assert 3 <= spec["speed_mps"] <= 4
        assert spec["distance_before_intersection_m"] >= 6 * spec["speed_mps"]
        assert -0.3 <= spec["lateral_offset_m"] <= 0.3
        assert abs(spec["heading_offset_deg"]) <= 4


@pytest.mark.parametrize("value", [True, "fast", -1, float("nan"), float("inf")])
def test_invalid_sample_values_cannot_reach_client(value):
    params = dict.fromkeys(run_verifai.FIELDS, 0)
    params["speed_mps"] = value
    with pytest.raises(ValueError, match="speed_mps"):
        parameters_to_spec(params)


@pytest.mark.parametrize("code,message", [
    (PARAMETERS.replace("param speed_mps = 3", ""), "param speed_mps"),
    (PARAMETERS.replace("param speed_mps = 3", "param speed_mps = -1"), "non-negative"),
    (PARAMETERS.replace("ego = new Object", "new Object"), "exactly one ego"),
    (PARAMETERS + "\nnew Object at (10, 0)\n", "additional actors"),
    (PARAMETERS + "\nterminate after 1 seconds\n", "initial-state constraints"),
    (PARAMETERS + "\nrequire always ego.position.x == 0\n", "initial-state constraints"),
    (PARAMETERS + "\nrecord final ego.position\n", "initial-state constraints"),
    ("behavior Drive():\n    wait\n" + PARAMETERS.replace(
        "ego = new Object", "ego = new Object with behavior Drive()"), "initial-state constraints"),
])
def test_unsupported_scenarios_fail_before_launch(code, message):
    with pytest.raises(ValueError, match=message):
        IntersectionScenicSampler.fromScenicCode(code)


def test_unsatisfiable_constraints_have_bounded_sampling():
    # An impossible random constraint, so Scenic must attempt rejection sampling.
    sampler = IntersectionScenicSampler.fromScenicCode(
        PARAMETERS + "\nx = Range(0, 1)\nrequire x < -1\n", maxIterations=3)
    with pytest.raises(RejectionException):
        sampler.nextSample()


def test_sample_only_is_replayable_and_needs_no_ranges_or_client(tmp_path, monkeypatch, capsys):
    config = yaml.safe_load((run_verifai.ROOT / "local/configs/verifai.yaml").read_text())
    del config["sampling"]["ranges"]
    config["sampling"]["count"] = 4
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config))

    def unexpected(*args, **kwargs):
        pytest.fail("sample-only must not create a server or simulation client")

    monkeypatch.setattr(verifai_server, "RolloutServer", unexpected)
    monkeypatch.setattr(verifai_server.subprocess, "Popen", unexpected)
    assert verifai_server.main(["--config", str(path), "--scenic", str(EXAMPLE),
                               "--sample-only", "--limit", "2"]) == 0
    resolved = yaml.safe_load(capsys.readouterr().out)
    assert resolved["rollouts"] == resolved["expanded_rollouts"] == sample_example(42, 2)
    assert resolved["sampling_info"]["sample_count"] == 2
    assert resolved["sampling_info"]["mode"] == "sample_only"
    assert resolved["sampling_info"]["sampler"] == "scenic"
    assert resolved["junction_entry"] == config["junction_entry"]


def test_scenic_and_feature_sampler_options_are_exclusive():
    with pytest.raises(SystemExit) as exc:
        verifai_server.main(["--scenic", str(EXAMPLE), "--sampler", "ce", "--sample-only"])
    assert exc.value.code == 2


def test_socket_feedback_updates_scenic_external_cross_entropy_sampler(monkeypatch, rollout_result):
    sampler = IntersectionScenicSampler.fromScenicCode(
        "param verifaiSamplerType = 'ce'\n" + PARAMETERS.replace(
            "param speed_mps = 3", "param speed_mps = VerifaiRange(3, 4)"))
    external = sampler.scenario.externalSampler
    distribution = external.sampler.domainSampler.cont_sampler.dist
    before = deepcopy(distribution)
    feedback = []
    original = sampler.nextSample

    def sample(score=None):
        feedback.append(score)
        return original(score)

    monkeypatch.setattr(sampler, "nextSample", sample)
    received = []

    def simulate(spec, index):
        received.append(spec)
        return rollout_result("collision" if index == 0 else "duration_reached")

    server = verifai_server.RolloutServer(sampler, None, 2, port=0, timeout=3,
                                          sampler_type="scenic", spec_for_sample=sampler.rollout_spec)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            client = pool.submit(run_client, server.port, simulate, timeout=3)
            assert server.run_server()[1] == -1
            server.index = 1
            assert server.run_server()[1] == 1
            assert client.result(timeout=3) == 2
        assert feedback == [None, -1]
        assert not np.array_equal(before, distribution)
        assert all(3 <= spec["speed_mps"] <= 4 for spec in received)
    finally:
        server.terminate()

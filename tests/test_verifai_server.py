"""Real VerifAI server, sockets and feedback, without CARLA or a GPU model."""
from concurrent.futures import ThreadPoolExecutor
import json
import random
import subprocess
import sys

import pytest

pytest.importorskip("verifai")
import numpy as np
import yaml

from local import run_verifai, verifai_client, verifai_server
from local.rollout_trace import load_rollout_result
from local.verifai_client import run_client


@pytest.fixture
def config():
    return yaml.safe_load((run_verifai.ROOT / "local/configs/verifai.yaml").read_text())


@pytest.mark.parametrize("summary,score", [
    ({"termination": "duration_reached"}, 1),
    ({"termination": "collision"}, -1),
    ({"termination": "duration_reached", "collision": {"actor": "wall"}}, -1),
])
def test_collision_specification(summary, score):
    assert verifai_server.collision_free({"summary": summary}) == score


@pytest.mark.parametrize("summary", [None, {}, {"termination": "error"},
    {"termination": "duration_reached", "error": "sensor timeout"}])
def test_incomplete_simulations_are_errors_not_scores(summary):
    with pytest.raises(RuntimeError):
        verifai_server.collision_free({"summary": summary})


@pytest.mark.parametrize("method", ["random", "halton", "ce"])
def test_socket_loop_feeds_collision_back_before_next_sample(config, monkeypatch, method, rollout_result):
    count, seed, ranges = run_verifai.sampling_settings(config)
    expected_random = run_verifai.sample_config(config)["rollouts"]
    random.seed(seed)
    np.random.seed(seed)
    sampler = verifai_server.make_sampler(ranges, method)
    updates = []
    original_update = sampler.update

    def update(sample, info, score):
        updates.append((run_verifai.rollout_spec(sample, ranges), score))
        original_update(sample, info, score)

    monkeypatch.setattr(sampler, "update", update)
    if method == "ce":
        distribution = sampler.domainSampler.cont_sampler.dist
        before = np.array(distribution).copy()
    received = []

    def simulate(spec, index):
        received.append(spec)
        return rollout_result("collision" if index == 0 else "duration_reached")

    server = verifai_server.RolloutServer(sampler, ranges, count, port=0, timeout=3,
                                          sampler_type=method)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            client = pool.submit(run_client, server.port, simulate, timeout=3)
            scores = []
            for index in range(count):
                server.index = index
                _, score, _ = server.run_server()
                scores.append(score)
            assert client.result(timeout=3) == count
        assert scores == [-1, 1]
        assert updates == [(received[0], -1)]
        for spec in received:
            for field, (low, high) in ranges.items():
                assert low <= spec[field] <= high
        if method == "random":
            assert received == expected_random
        if method == "ce":
            assert not np.array_equal(before, distribution)
    finally:
        server.terminate()


def test_missing_client_times_out(config):
    _, _, ranges = run_verifai.sampling_settings(config)
    server = verifai_server.RolloutServer(
        verifai_server.make_sampler(ranges, "random"), ranges, 1, port=0, timeout=0.05)
    try:
        with pytest.raises(TimeoutError, match="waiting for the CarlaMayo client"):
            server.run_server()
    finally:
        server.terminate()


def test_full_recorded_trace_reaches_custom_monitor_and_drives_feedback(
        config, recorded_rollout, monkeypatch):
    _, _, ranges = run_verifai.sampling_settings(config)
    recordings = [recorded_rollout(str(index), steps=40, speed=speed)
                  for index, speed in enumerate((7.0, 3.0))]
    sampler = verifai_server.make_sampler(ranges, "ce")
    feedback = []
    original_next = sampler.nextSample

    def sample(score=None):
        feedback.append(score)
        return original_next(score)

    monkeypatch.setattr(sampler, "nextSample", sample)
    observed = []

    def specification(result):
        observed.append(result)
        # Both summaries report no collision; only the time-series speeds violate this spec.
        return min(5.0 - row["speed"] for row in result["trace"]["samples"])

    server = verifai_server.RolloutServer(
        sampler, ranges, 2, port=0, timeout=5, sampler_type="ce",
        monitor=verifai_server.specification_monitor(specification))

    def send_fragments(connection, message):
        payload = (json.dumps(message, allow_nan=False) + "\n").encode()
        assert len(payload) > server.bufsize
        for start in range(0, len(payload), 37):
            connection.sendall(payload[start:start + 37])

    monkeypatch.setattr(verifai_client, "send_json", send_fragments)

    def simulate(spec, index):
        directory, expected = recordings[index]
        return load_rollout_result(directory, expected["summary"])

    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            client = pool.submit(run_client, server.port, simulate, timeout=5)
            assert server.run_server()[1] == -2.0
            server.index = 1
            assert server.run_server()[1] == 2.0
            assert client.result(timeout=5) == 2
        assert observed == [expected for _, expected in recordings]
        assert server.last_result == recordings[-1][1]
        assert feedback == [None, -2.0]
    finally:
        server.terminate()


def test_server_rejects_summary_only_results_before_custom_monitor(config):
    _, _, ranges = run_verifai.sampling_settings(config)

    def unexpected(result):
        pytest.fail("A missing trace must not reach the specification monitor")

    server = verifai_server.RolloutServer(
        verifai_server.make_sampler(ranges, "random"), ranges, 1, port=0, timeout=3,
        monitor=verifai_server.specification_monitor(unexpected))
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            client = pool.submit(run_client, server.port, lambda spec, index: {
                "summary": {"termination": "duration_reached"}, "trace": None}, 3)
            with pytest.raises(ValueError, match="Missing full rollout trace"):
                server.run_server()
            assert client.result(timeout=3) == 1
    finally:
        server.terminate()


def test_coordinator_port_can_be_reused_immediately(config, rollout_result):
    _, _, ranges = run_verifai.sampling_settings(config)
    port = 0
    for _ in range(2):
        server = verifai_server.RolloutServer(
            verifai_server.make_sampler(ranges, "random"), ranges, 1, port=port, timeout=3)
        port = server.port
        try:
            with ThreadPoolExecutor(max_workers=1) as pool:
                client = pool.submit(run_client, port,
                                     lambda spec, index: rollout_result(), 3)
                assert server.run_server()[1] == 1
                assert client.result(timeout=3) == 1
        finally:
            server.terminate()


@pytest.mark.parametrize("failure", [None, "simulation", "startup", "trace"])
@pytest.mark.parametrize("use_scenic", [False, True])
def test_launcher_runs_client_and_records_results_or_errors(tmp_path, config, monkeypatch, failure,
                                                           use_scenic, recorded_rollout):
    # Replace only GPU simulation with a real subprocess speaking the same protocol.
    # The production coordinator still chooses its samples, launches/waits for a
    # client, evaluates its monitor, and writes its normal replay/results files.
    real_popen = subprocess.Popen
    children = []
    fixtures = []
    for index in range(2):
        directory, result = recorded_rollout(str(index), termination=(
            "collision" if index == 0 else "duration_reached"))
        fixtures.append(result)
    fixture_root = directory.parent
    if failure == "trace":
        (fixture_root / "0/predictions.jsonl").write_text("")
    code = '''
import json
from pathlib import Path
import sys
from local.verifai_client import run_client
from local.rollout_trace import load_rollout_result
options = dict(zip(sys.argv[1::2], sys.argv[2::2]))
def simulate(spec, index):
    FAILURE
    path = Path(options["--fixtures"]) / str(index)
    return load_rollout_result(path, json.loads((path / "summary.json").read_text()))
run_client(int(options["--verifai-port"]), simulate, timeout=3)
'''.replace("FAILURE", "raise RuntimeError('sensor timeout')" if failure == "simulation" else "pass")
    if failure == "startup":
        code = "raise SystemExit(7)"

    def start_client(command, **kwargs):
        assert command[:3] == [str(run_verifai.ROOT / ".venv/bin/python"), "-m", "local.verifai_client"]
        child = real_popen([sys.executable, "-c", code, *command[3:],
                           "--fixtures", str(fixture_root)], **kwargs)
        children.append(child)
        return child

    monkeypatch.setattr(verifai_server.subprocess, "Popen", start_client)
    output = tmp_path / "run"
    arguments = ["--output", str(output), "--duration", "0.1", "--verifai-port", "0", "--timeout", "3"]
    scenario = run_verifai.ROOT / "local/scenarios/intersection.scenic"
    if use_scenic:
        arguments.extend(("--scenic", str(scenario)))
    if failure:
        message = {"simulation": "sensor timeout", "startup": "exited with code 7",
                   "trace": "every model prediction"}[failure]
        with pytest.raises(RuntimeError, match=message):
            verifai_server.main(arguments)
    else:
        assert verifai_server.main(arguments) == 0
    assert len(children) == 1 and children[0].poll() is not None
    status = json.loads((output / "verifai-summary.json").read_text())
    results = [json.loads(line) for line in (output / "verifai-results.jsonl").read_text().splitlines()]
    resolved = json.loads((output / "config.json").read_text())
    if use_scenic:
        assert resolved["sampling_info"]["sampler"] == "scenic"
        assert "ranges" not in resolved["sampling"]
        assert (output / "scenario.scenic").read_bytes() == scenario.read_bytes()
    if failure:
        assert status["status"] == "error"
        assert status["completed_rollouts"] == status["violations"] == 0
        assert results == []
        assert message in status["error"]
    else:
        assert status["status"] == "completed"
        assert status["completed_rollouts"] == 2 and status["violations"] == 1
        assert [result["score"] for result in results] == [-1, 1]
        assert [result["passed"] for result in results] == [False, True]
        assert [result["trace"] for result in results] == [fixture["trace"] for fixture in fixtures]
        assert [result["summary"] for result in results] == [fixture["summary"] for fixture in fixtures]
        if use_scenic:
            from local.scenic_sampling import IntersectionScenicSampler

            random.seed(config["sampling"]["seed"])
            np.random.seed(config["sampling"]["seed"])
            sampler = IntersectionScenicSampler.fromScenario(str(scenario))
            expected = [sampler.rollout_spec(sampler.nextSample()) for _ in range(2)]
        else:
            expected = run_verifai.sample_config(config)["rollouts"]
        assert resolved["rollouts"] == resolved["expanded_rollouts"] == expected
        assert resolved["duration_s"] == 0.1
        assert [result["requested"] for result in results] == expected

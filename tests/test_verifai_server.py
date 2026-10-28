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

from local import run_verifai, verifai_server
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
    assert verifai_server.collision_free(summary) == score


@pytest.mark.parametrize("summary", [None, {}, {"termination": "error"},
    {"termination": "duration_reached", "error": "sensor timeout"}])
def test_incomplete_simulations_are_errors_not_scores(summary):
    with pytest.raises(RuntimeError):
        verifai_server.collision_free(summary)


@pytest.mark.parametrize("method", ["random", "halton", "ce"])
def test_socket_loop_feeds_collision_back_before_next_sample(config, monkeypatch, method):
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
        return {"termination": "collision" if index == 0 else "duration_reached"}

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


def test_coordinator_port_can_be_reused_immediately(config):
    _, _, ranges = run_verifai.sampling_settings(config)
    port = 0
    for _ in range(2):
        server = verifai_server.RolloutServer(
            verifai_server.make_sampler(ranges, "random"), ranges, 1, port=port, timeout=3)
        port = server.port
        try:
            with ThreadPoolExecutor(max_workers=1) as pool:
                client = pool.submit(run_client, port,
                                     lambda spec, index: {"termination": "duration_reached"}, 3)
                assert server.run_server()[1] == 1
                assert client.result(timeout=3) == 1
        finally:
            server.terminate()


@pytest.mark.parametrize("failure", [None, "simulation", "startup"])
def test_launcher_runs_client_and_records_results_or_errors(tmp_path, config, monkeypatch, failure):
    # Replace only GPU simulation with a real subprocess speaking the same protocol.
    # The production coordinator still chooses its samples, launches/waits for a
    # client, evaluates its monitor, and writes its normal replay/results files.
    real_popen = subprocess.Popen
    children = []
    code = '''
import sys
from local.verifai_client import run_client
options = dict(zip(sys.argv[1::2], sys.argv[2::2]))
def simulate(spec, index):
    FAILURE
    return {"termination": "collision" if index == 0 else "duration_reached"}
run_client(int(options["--verifai-port"]), simulate, timeout=3)
'''.replace("FAILURE", "raise RuntimeError('sensor timeout')" if failure == "simulation" else "pass")
    if failure == "startup":
        code = "raise SystemExit(7)"

    def start_client(command, **kwargs):
        assert command[:3] == [str(run_verifai.ROOT / ".venv/bin/python"), "-m", "local.verifai_client"]
        child = real_popen([sys.executable, "-c", code, *command[3:]], **kwargs)
        children.append(child)
        return child

    monkeypatch.setattr(verifai_server.subprocess, "Popen", start_client)
    output = tmp_path / "run"
    arguments = ["--output", str(output), "--duration", "0.1", "--verifai-port", "0", "--timeout", "3"]
    if failure:
        message = "sensor timeout" if failure == "simulation" else "exited with code 7"
        with pytest.raises(RuntimeError, match=message):
            verifai_server.main(arguments)
    else:
        assert verifai_server.main(arguments) == 0
    assert len(children) == 1 and children[0].poll() is not None
    status = json.loads((output / "verifai-summary.json").read_text())
    results = [json.loads(line) for line in (output / "verifai-results.jsonl").read_text().splitlines()]
    resolved = json.loads((output / "config.json").read_text())
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
        expected = run_verifai.sample_config(config)["rollouts"]
        assert resolved["rollouts"] == resolved["expanded_rollouts"] == expected
        assert resolved["duration_s"] == 0.1
        assert [result["requested"] for result in results] == expected

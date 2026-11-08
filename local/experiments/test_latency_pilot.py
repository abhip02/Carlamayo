import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

from local.experiments import run_latency_pilot as pilot
from local.experiments.test_resource_lease import MODEL, RENDER


def setup_pilot(tmp_path, monkeypatch, jobs):
    code = tmp_path / 'local/experiments'
    (code / 'presets').mkdir(parents=True)
    (code / 'projected_server.sh').write_text('#!/bin/bash\nexit 0\n')
    manifest = tmp_path / 'manifest.json'
    manifest.write_text('{}')
    protocol = dict(source_sha256={}, scenario_manifest='manifest.json',
        scenario_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(), jobs=jobs,
        resource_requirements={'container': 'test-owned-container'})
    path = code / 'presets/latency-pilot.json'
    path.write_text(json.dumps(protocol))
    monkeypatch.setattr(pilot, '__file__', str(code / 'run_latency_pilot.py'))
    monkeypatch.setattr(pilot, 'require_idle', lambda *args: [{'test_resources': True}])
    monkeypatch.setattr(pilot.shutil, 'disk_usage', lambda p: SimpleNamespace(free=8*1024**3))
    original = pilot.subprocess.check_output
    monkeypatch.setattr(pilot.subprocess, 'check_output', lambda cmd, **kw:
        'test-commit\n' if cmd[:2] == ['git', 'rev-parse'] else original(cmd, **kw))
    monkeypatch.setenv('CARLAMAYO_MODEL_GPUS', MODEL)
    monkeypatch.setenv('CARLAMAYO_RENDER_GPU', RENDER)
    monkeypatch.setattr(sys, 'argv', ['pilot', '--output', str(tmp_path / 'outputs/pilot')])
    return path


def test_timed_out_and_failed_jobs_are_recorded_and_do_not_hide_later_outcomes(tmp_path, monkeypatch):
    jobs = []
    for name, program, timeout in [('timeout', 'import time; time.sleep(5)', .05),
                                   ('failure', 'raise SystemExit(7)', 5),
                                   ('success', 'print("done")', 5)]:
        jobs.append(dict(name=name, output=f'outputs/case/{name}', timeout_seconds=timeout,
            argv=[sys.executable, '-c', program, '--output', 'replaced-by-driver']))
    setup_pilot(tmp_path, monkeypatch, jobs)
    with pytest.raises(SystemExit) as exc:
        pilot.main()
    assert exc.value.code == 1
    status = json.loads((tmp_path / 'outputs/pilot/status.json').read_text())
    assert status['state'] == 'completed' and status['planned'] == 3
    assert [r['name'] for r in status['results']] == ['timeout', 'failure', 'success']
    assert status['results'][0]['timed_out']
    assert [r['returncode'] for r in status['results'][1:]] == [7, 0]
    assert all(r['cleanup_returncode'] == 0 for r in status['results'])


def test_source_mismatch_blocks_before_output_or_any_job(tmp_path, monkeypatch):
    path = setup_pilot(tmp_path, monkeypatch, [])
    (tmp_path / 'manifest.json').write_text('{"changed":true}')
    with pytest.raises(RuntimeError, match='manifest changed'):
        pilot.main()
    assert not (tmp_path / 'outputs').exists()


def test_gpu_block_releases_leases_and_creates_no_capture(tmp_path, monkeypatch):
    setup_pilot(tmp_path, monkeypatch, [])
    def busy(*args):
        raise RuntimeError('GPU is occupied')
    monkeypatch.setattr(pilot, 'require_idle', busy)
    with pytest.raises(RuntimeError, match='occupied'):
        pilot.main()
    assert not (tmp_path / 'outputs').exists()
    locks = tmp_path.parent / '.carlamayo-resource-locks'
    assert all(p.stat().st_size == 0 for p in locks.iterdir())

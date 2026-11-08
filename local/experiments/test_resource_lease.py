import socket

import pytest

from local.experiments import resource_lease as lease


MODEL = 'GPU-00000000-0000-0000-0000-000000000001'
RENDER = 'GPU-00000000-0000-0000-0000-000000000002'
THIRD = 'GPU-00000000-0000-0000-0000-000000000003'


def test_competing_lease_cannot_claim_same_port_and_releases_after_failure(tmp_path):
    with lease.acquire(tmp_path, MODEL, RENDER):
        with pytest.raises(RuntimeError, match='already leased'):
            with lease.acquire(tmp_path, MODEL, THIRD):
                pytest.fail('Competing worker acquired resources')
    with lease.acquire(tmp_path, MODEL, THIRD):
        assert any(p.stat().st_size for p in tmp_path.iterdir())
    assert all(p.stat().st_size == 0 for p in tmp_path.iterdir())


@pytest.mark.parametrize('model,renderer', [(MODEL, MODEL), ('0', RENDER), (MODEL + ',1', RENDER), (None, RENDER)])
def test_ambiguous_or_shared_device_configuration_is_rejected(tmp_path, model, renderer):
    with pytest.raises(RuntimeError):
        with lease.acquire(tmp_path, model, renderer):
            pytest.fail('Bad GPU configuration acquired a lease')


def test_occupied_gpu_blocks_work_without_displacing_it(monkeypatch):
    monkeypatch.setattr(lease, 'gpu_snapshot', lambda gpu: dict(uuid=gpu, name='NVIDIA RTX A5000', memory_used_mib=1024))
    with pytest.raises(RuntimeError, match='GPU is occupied'):
        lease.require_idle(MODEL, RENDER)


def test_live_port_blocks_an_otherwise_idle_configuration(monkeypatch):
    monkeypatch.setattr(lease, 'gpu_snapshot', lambda gpu: dict(uuid=gpu, name='NVIDIA RTX A5000', memory_used_mib=0))
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        port = listener.getsockname()[1]
        with pytest.raises(RuntimeError, match='CARLA port'):
            lease.require_idle(MODEL, RENDER, ports=(port,))
    assert len(lease.require_idle(MODEL, RENDER, ports=())) == 2

"""Test the actual inference boundary without loading weights or a simulator."""
import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from local.experiments.native_policy import resolve_sampling, camera_postprocessing
from local.experiments.inference_probe import Probe
from local.experiments import inference_probe as native
from local.experiments.async_projected_probe import AsyncSingleSampleProbe


@pytest.mark.parametrize('mode,expected', [
    ('fixed', (43, 17)), ('advancing_separate', (50, 24)), ('advancing_joint', (50, None))])
def test_schedule_uses_request_count_and_preserves_requested_configuration(mode, expected):
    options = dict(seed=43, flow_seed=17, seed_policy=mode)
    before = copy.deepcopy(options)
    effective, trace = resolve_sampling(options, dict(index=92, inference_index=7))
    assert (effective.get('seed'), effective.get('flow_seed')) == expected
    assert trace['effective_seed'] == expected[0]
    assert options == before
    assert resolve_sampling(options, dict(index=999, inference_index=7)) == (effective, trace)


@pytest.mark.parametrize('options,context', [
    ({'seed_policy':'unknown'}, {}), ({'seed':True}, {}), ({'flow_seed':-1}, {}),
    ({'seed_policy':'advancing_joint'}, {}),
    ({'seed_policy':'advancing_joint'}, {'inference_index':True}),
    ({'seed_policy':'advancing_joint'}, {'inference_index':-1}),
    ({'seed_policy':'advancing_joint','seed':2**32-1}, {'inference_index':1}),
    ({'seed_policy':'advancing_separate'}, {'inference_index':0}),
    ({'seed_policy':'advancing_separate','flow_seed':2**32-1}, {'inference_index':1}),
    ({'seed_policy':'advancing_joint','num_samples':2}, {'inference_index':0}),
    ({'seed_policy':'advancing_joint','speed_selection_attempts':2}, {'inference_index':0})])
def test_invalid_schedule_fails_before_inference(options, context):
    with pytest.raises(ValueError):
        resolve_sampling(options, context)


def test_async_ordinal_advances_even_when_control_frame_does_not(tmp_path, monkeypatch):
    monkeypatch.setattr(Probe, '__init__', lambda self,*a: None)
    probe = AsyncSingleSampleProbe(None, tmp_path)
    try:
        ctx = dict(index=90, time_s=9., world_pose={'yaw':20.})
        requests = [probe.make_request(None, {}, ctx, 'Turn left.', {}, False) for _ in range(3)]
        assert [r['options']['inference_index'] for r in requests] == [0,1,2]
        assert [r['context']['index'] for r in requests] == [90,90,90]
        assert all(r['context'] == ctx for r in requests)
        assert 'inference_index' not in ctx
    finally:
        probe.events.close()


@pytest.mark.parametrize('mode', ['fixed','advancing_separate','advancing_joint'])
@pytest.mark.parametrize('vqa', [False,True])
def test_probe_replays_effective_rng_without_double_advancing_or_vqa_interference(tmp_path, monkeypatch, mode, vqa):
    monkeypatch.setattr(torch.cuda, 'device_count', lambda: 0)
    # Invoke real Probe.evaluate and real flow_sample. Only the expensive model
    # is replaced; CPU torch random draws stand in for text and flow sampling.
    probe = Probe.__new__(Probe)
    probe.output = tmp_path
    probe.visual_signal, probe.visual_signal_index = None, -1000
    probe.model = SimpleNamespace(tokenizer=SimpleNamespace(
        convert_tokens_to_ids=lambda token:1, decode=lambda *a,**kw:'trajectory'))
    probe.original_flow_sample = lambda: torch.rand(64,3)
    calls = []
    def infer(adapter, model, processor, data, **kwargs):
        calls.append(kwargs)
        torch.rand(11)  # Language consumes the stream before motion.
        points = probe.flow_sample()
        probe.trace.update(candidate_generated_tokens=[[1]], candidate_markers=[True],
                           candidate_velocities=[[1.]*64], candidate_accelerations=[[0.]*64],
                           decoder_estimated_speed_mps=1., decoder_used_speed_mps=1.)
        return points[None,None,None], {}
    monkeypatch.setattr(native.Alpamayo15Adapter, 'run_inference', infer)
    class Adapter:
        probe_processor = None
        def run_vqa(self,*args):
            torch.rand(150)
            return 'green'
        def extract_answer_text(self, answer): return answer
        def extract_cot_text(self, extra): return ''
    data = {'camera_indices':torch.tensor([0]), 'image_frames':torch.zeros(1,4,3,2,2)}
    context = dict(index=90, inference_index=7)
    options = dict(seed=43, flow_seed=17, seed_policy=mode, verify_signal=vqa)
    pred, _ = probe.evaluate(Adapter(), data, context, 'Turn left.', options, 'first')
    saved = json.loads((tmp_path/'first.json').read_text())
    assert saved['options'] == options
    second, _ = probe.evaluate(Adapter(), data, saved['context'], saved['instruction'], saved['options'], 'second')
    assert torch.equal(pred, second)
    effective, trace = resolve_sampling(options, context)
    assert saved['sampling_policy'] == trace
    torch.manual_seed(effective['seed'])
    torch.rand(11)
    if 'flow_seed' in effective: torch.manual_seed(effective['flow_seed'])
    np.testing.assert_array_equal(pred.numpy()[0,0,0], torch.rand(64,3).numpy())
    assert len(calls) == 2
    # A nearby raw model-output change must remain visible in the returned plan.
    probe.original_flow_sample = lambda: torch.rand(64,3) + .1
    changed, _ = probe.evaluate(Adapter(), data, context, 'Turn left.', options, 'changed')
    np.testing.assert_allclose(changed.numpy() - pred.numpy(), .1, atol=1e-7)


@pytest.mark.parametrize('value', [True,False])
@pytest.mark.parametrize('fail', [True,False])
def test_camera_flag_reaches_sensor_blueprint_and_restores_on_failure(monkeypatch, value, fail):
    from module import carla_interface as ci
    from module.adapters._rigs import FOUR_CAMERA_RIG
    before = copy.deepcopy(FOUR_CAMERA_RIG)
    attrs = []
    sensor = SimpleNamespace(listen=lambda callback:None)
    class Blueprint:
        def set_attribute(self, name, val): attrs.append((name,val))
    interface = ci.CARLAInterface()
    interface.ego_vehicle = object()
    interface.world = SimpleNamespace(
        get_blueprint_library=lambda:SimpleNamespace(find=lambda name:Blueprint()),
        spawn_actor=lambda *a,**kw:sensor)
    monkeypatch.setattr(ci, 'camera_transforms', lambda *a:{name:object() for name in FOUR_CAMERA_RIG})
    monkeypatch.setattr(ci.time, 'sleep', lambda *a:None)
    old = ci.cfg.CAMERA_ENABLE_POSTPROCESS_EFFECTS
    try:
        with camera_postprocessing(ci.cfg, {'camera_enable_postprocess_effects':value}):
            interface.setup_cameras()
            if fail: raise RuntimeError('capture error')
    except RuntimeError:
        assert fail
    assert ci.cfg.CAMERA_ENABLE_POSTPROCESS_EFFECTS == old
    assert [v for k,v in attrs if k=='enable_postprocess_effects'] == [str(value)]*4
    assert FOUR_CAMERA_RIG == before


@pytest.mark.parametrize('value', ['off',0,None])
def test_camera_rejects_ambiguous_override(value):
    cfg = SimpleNamespace(CAMERA_ENABLE_POSTPROCESS_EFFECTS=True)
    with pytest.raises(ValueError), camera_postprocessing(cfg, {'camera_enable_postprocess_effects':value}):
        pass
    assert cfg.CAMERA_ENABLE_POSTPROCESS_EFFECTS is True


@pytest.mark.parametrize('arm', ['fixed','advancing_separate','advancing_joint'])
@pytest.mark.parametrize('camera', [None,'on','off'])
def test_suite_writes_actual_capture_options_before_server_start(tmp_path, monkeypatch, arm, camera):
    from local.experiments import run_projection_suite as suite
    from pathlib import Path
    root = Path(suite.__file__).resolve().parents[2]
    manifest = root/'local/experiments/presets/ten-driving-traffic-scenes.json'
    output = tmp_path/'capture'
    args = ['suite','--output',str(output),'--manifest',str(manifest),'--cases','2',
            '--steering-modes','native','--async-inference',
            '--async-fresh-planning-interval-s','0.5','--seed-policy',arm,
            '--server-script',str(root/'local/experiments/projected_server.sh')]
    if camera is not None: args += ['--camera-postprocessing',camera]
    monkeypatch.setattr(suite.sys,'argv',args)
    monkeypatch.setattr(suite.subprocess,'check_output',lambda *a,**kw:'frozen-source-commit\n')
    class BeforeServerStart(Exception): pass
    def no_server(command,**kwargs):
        if command[-1]=='start': raise BeforeServerStart()
        assert command[-1]=='stop'
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(suite.subprocess,'run',no_server)
    with pytest.raises(BeforeServerStart): suite.main()
    options = json.loads((output/'02-intersection-left-native-options.json').read_text())
    assert options['seed_policy']==arm and options['steering_mode']=='native'
    assert options['num_samples']==1
    assert options.get('camera_enable_postprocess_effects') == (None if camera is None else camera=='on')


def test_existing_shared_gpu_lease_blocks_overlap_and_releases_without_writing(tmp_path,monkeypatch):
    from local.experiments import replay_seed_policy as replay
    import fcntl
    gpu = 'GPU-216b01e6-03fa-563d-0ff0-739ef6f558f8'
    lock = tmp_path/f'gpu-{gpu}.lock'
    lock.write_text('preserved shared metadata')
    monkeypatch.setattr(replay,'SHARED_LOCKS',tmp_path)
    monkeypatch.setattr(replay,'require_idle_gpu',lambda gpu:{'uuid':gpu})
    with replay.gpu_lease(gpu):
        with lock.open('r+') as other, pytest.raises(BlockingIOError):
            fcntl.flock(other,fcntl.LOCK_EX|fcntl.LOCK_NB)
    with lock.open('r+') as other: fcntl.flock(other,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert lock.read_text()=='preserved shared metadata'


def test_gpu_preflight_never_loads_model_on_occupied_gpu(monkeypatch):
    from local.experiments import replay_seed_policy as replay
    gpu = 'GPU-216b01e6-03fa-563d-0ff0-739ef6f558f8'
    calls = []
    def occupied(command,**kw):
        calls.append(command)
        return gpu+', 12345\n'
    monkeypatch.setattr(replay.subprocess,'check_output',occupied)
    with pytest.raises(RuntimeError,match='GPU occupied'): replay.require_idle_gpu(gpu)
    assert len(calls)==1


def test_input_hash_mismatch_fails_and_archive_fallback_verifies(tmp_path):
    from local.experiments import replay_seed_policy as replay
    import gzip, hashlib
    path = tmp_path/'missing.pt'
    data = b'untouched fixture bytes'
    archive = tmp_path/'fixture.pt.gz'
    archive.write_bytes(gzip.compress(data))
    row = dict(source_input=str(path),archive=str(archive),raw_sha256=hashlib.sha256(data).hexdigest())
    assert replay.input_bytes(row)==data
    path.write_bytes(b'wrong input')
    with pytest.raises(ValueError,match='hash mismatch'): replay.input_bytes(row)

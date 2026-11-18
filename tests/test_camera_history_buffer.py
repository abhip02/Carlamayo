from collections import deque

import numpy as np
import torch

from local.run_intersection import stack_camera_history
from module.adapters.alpamayo_1_5 import Alpamayo15Adapter


def test_camera_and_time_order_matches_original_stack_across_ticks():
    rng = np.random.default_rng(19)
    frames = deque(maxlen=4)
    for _ in range(4):
        frames.append(rng.integers(0, 256, (4, 12, 20, 3), dtype=np.uint8))
    buffer = None
    identity = None
    for _ in range(7):
        expected = np.stack(frames, axis=1)
        buffer = stack_camera_history(frames, buffer)
        np.testing.assert_array_equal(buffer, expected)
        assert buffer.dtype == expected.dtype and buffer.strides == expected.strides
        if identity is None:
            identity = id(buffer)
        assert id(buffer) == identity
        frames.append(rng.integers(0, 256, (4, 12, 20, 3), dtype=np.uint8))


def test_adapter_inputs_equal_original_and_prior_request_is_not_overwritten():
    adapter = Alpamayo15Adapter()
    rng = np.random.default_rng(23)
    frames = deque([rng.integers(0, 256, (4, 12, 20, 3), dtype=np.uint8)
                    for _ in range(4)], maxlen=4)
    xyz = rng.normal(size=(16, 3)).astype(np.float32)
    rotation = np.repeat(np.eye(3, dtype=np.float32)[None], 16, axis=0)
    buffer = stack_camera_history(frames)
    data = adapter.prepare_model_input(buffer, xyz, rotation, 123456)
    expected = adapter.prepare_model_input(np.stack(frames, axis=1), xyz, rotation, 123456)
    assert data.keys() == expected.keys()
    for name in data:
        assert torch.equal(data[name], expected[name]), name
    previous = data['image_frames'].clone()
    frames.append(np.zeros_like(frames[-1]))
    assert stack_camera_history(frames, buffer) is buffer
    assert torch.equal(data['image_frames'], previous)
    next_data = adapter.prepare_model_input(buffer, xyz, rotation, 223456)
    next_expected = adapter.prepare_model_input(np.stack(frames, axis=1), xyz, rotation, 223456)
    assert torch.equal(next_data['image_frames'], next_expected['image_frames'])
    assert not torch.equal(next_data['image_frames'], previous)

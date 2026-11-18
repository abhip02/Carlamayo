"""Check the two isolated changes before spending GPU time on the panel."""
import unittest
from unittest.mock import patch

from local.experiments import resume_fixed_cases as resumed


class ResumeTests(unittest.TestCase):
    def test_loader_restored_after_capture_error(self):
        from local.experiments import nf4_startup_loader
        original = resumed.policy.bridge
        with self.assertRaisesRegex(RuntimeError, 'capture failure'):
            with resumed.repaired_bridge():
                self.assertIs(resumed.policy.bridge, nf4_startup_loader)
                raise RuntimeError('capture failure')
        self.assertIs(resumed.policy.bridge, original)

    def test_startup_budget_increases_but_runtime_queries_do_not_gate(self):
        calls = []
        def observe(gpus, minimums):
            calls.append(minimums)
            return dict(zip(gpus, minimums))
        gpus = [resumed.MODEL_GPU, resumed.RENDER_GPU]
        with patch.object(resumed.guard, 'bounded_memory', observe):
            with resumed.repaired_memory_gate():
                resumed.guard.bounded_memory(gpus, [10., 9.8])
                resumed.guard.bounded_memory(gpus, [0., 0.])
            self.assertIs(resumed.guard.bounded_memory, observe)
        self.assertEqual(calls, [[11., 9.8], [0., 0.]])

    def test_gate_restored_on_error(self):
        original = resumed.guard.bounded_memory
        with self.assertRaises(ValueError):
            with resumed.repaired_memory_gate():
                raise ValueError('stop')
        self.assertIs(resumed.guard.bounded_memory, original)

    def test_controller_or_loader_amendment_rejected(self):
        for key, value in [('controller_changed', True), ('allocation_fraction', .41),
                           ('camera_pause_seconds', 1)]:
            with self.subTest(key=key):
                modified = {**resumed.amendment(), key: value}
                with self.assertRaises(ValueError):
                    resumed.verify_amendment({'resume_amendment': modified})


if __name__ == '__main__':
    unittest.main()

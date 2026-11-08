"""Checks that frozen jobs and cooperative resource ownership cannot drift silently."""
import json
import pytest

from local.experiments.model_score_benchmark import sha256
from local.experiments.nf4_closed_loop import verify_manifest, qualified_map
from local.experiments.resource_lease import lease


def test_frozen_job_rejects_config_and_source_mutation(tmp_path):
    config, options, source = [tmp_path / n for n in ("job.yaml", "options.json", "source.py")]
    for path in (config, options, source):
        path.write_text("original")
    manifest = tmp_path / "jobs.json"
    manifest.write_text(json.dumps(dict(jobs=[dict(name="job", config_sha256=sha256(config),
        options_sha256=sha256(options))], source_sha256={"source.py": sha256(source)})))
    verify_manifest(manifest, tmp_path / "job", config, options, root=tmp_path)
    config.write_text("changed")
    with pytest.raises(ValueError, match="Frozen job changed"):
        verify_manifest(manifest, tmp_path / "job", config, options, root=tmp_path)
    config.write_text("original")
    source.write_text("changed")
    with pytest.raises(ValueError, match="Frozen source changed"):
        verify_manifest(manifest, tmp_path / "job", config, options, root=tmp_path)


def test_lease_conflict_releases_partial_acquisition(tmp_path):
    with lease(["GPU-b"], lock_root=tmp_path):
        with pytest.raises(RuntimeError, match="already leased"):
            with lease(["GPU-a", "GPU-b"], lock_root=tmp_path):
                pytest.fail("conflicting lease admitted")
        with lease(["GPU-a"], lock_root=tmp_path):
            pass
    with lease(["GPU-a", "GPU-b"], lock_root=tmp_path):
        pass


def test_live_map_requires_matching_qualified_layout_and_equivalence(tmp_path):
    mapping = {}
    for i in range(36):
        mapping[f"vlm.model.language_model.layers.{i}"] = i % 3
        mapping[f"expert.layers.{i}"] = i % 3
    layout, precision, verification = [tmp_path / name for name in ("map.json", "precision.json", "verification.json")]
    layout.write_text(json.dumps(mapping))
    precision.write_text(json.dumps(dict(device_map=mapping)))
    verification.write_text(json.dumps(dict(identical_tokens=True, max_coordinate_error_m=0.)))
    assert qualified_map(layout, precision, verification) == mapping
    mapping["expert.layers.0"] = 2
    layout.write_text(json.dumps(mapping))
    with pytest.raises(ValueError, match="differs from the qualified"):
        qualified_map(layout, precision, verification)
    precision.write_text(json.dumps(dict(device_map=mapping)))
    with pytest.raises(ValueError, match="must share a device"):
        qualified_map(layout, precision, verification)

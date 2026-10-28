"""Update the sampling-status slide and append three VerifAI summary slides.

Usage: python add_verifai_update.py OUTPUT_DIRECTORY
The directory must contain the latest Drive deck as original.pptx.
Existing slide/media parts are preserved except slide 13 and its speaker notes.
"""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import sys
import zipfile

from pptx import Presentation
from lxml import etree

work = Path(sys.argv[1]).resolve()
source = work / "original.pptx"
prs = Presentation(source)
assert len(prs.slides) == 17, "Review the latest source deck before extending it."
source_titles = [slide.shapes.title.text for slide in prs.slides]
exemplar = prs.slides[1]
assert exemplar.shapes.title.text == "Alpamayo 1.5: two-GPU sharding"
assert len(exemplar.shapes) == 2 and all(s.has_text_frame for s in exemplar.shapes)


def replace_text(shape, texts):
    """Preserve the exemplar's native bullets and per-paragraph run styling."""
    assert len(shape.text_frame.paragraphs) == len(texts)
    for paragraph, text in zip(shape.text_frame.paragraphs, texts):
        assert len(paragraph.runs) == 1
        paragraph.runs[0].text = text


def fill(slide, title, items, notes):
    replace_text(slide.shapes.title, [title])
    replace_text(slide.shapes[1], items)
    slide.notes_slide.notes_text_frame.text = notes


def append(title, items, notes):
    slide = prs.slides.add_slide(exemplar.slide_layout)
    for shape in list(slide.shapes):
        shape._element.getparent().remove(shape._element)
    for shape in exemplar.shapes:
        slide.shapes._spTree.insert_element_before(deepcopy(shape._element), "p:extLst")
    fill(slide, title, items, notes)
    return slide


status = prs.slides[12]
assert status.shapes.title.text == "Initial states: current grid and probability model"
fill(status, "Initial states: grid baseline and VerifAI sampling", [
    "Original runner: explicit starts or a Cartesian grid. VerifAI now adds seeded random sampling",
    "Configured grid: distance {10, 20, 30} m × speed {0, 3, 6} m/s × heading {−10°, 0°, +10°}; lateral offset 0. Full grid not yet run",
    "New example: independent continuous uniforms for distance 20–25 m, speed 3–4 m/s and heading −5° to +5°; lateral offset fixed at 0",
    "Save samples and scene seed, then reuse the same pre-roll and rollout loop. Two new 1 s sampled runs verified the launcher",
], "Updated 2026-10-28 to replace the now-obsolete statement that sampling is unimplemented. "
   "The original 27-state Cartesian grid remains unexecuted. The two original explicit demo runs "
   "were 8 s each; the new VerifAI integration checks were 1 s each. These are distinct experiments. "
   "The original illustrative discrete-uniform proposal is superseded by the continuous-uniform "
   "example in local/configs/verifai.yaml. Sources: local/run_verifai.py, local/configs/verifai.yaml, "
   "outputs/verifai-integration-smoke/batch-summary.json. Code commit 0fa67d6.")

append("VerifAI: sample initial states, then launch rollouts", [
    "local/run_verifai.py uses VerifAI 2.2.0 to sample initial states and launch the existing batch runner",
    "Sample distance, speed and heading from declared uniform ranges; equal bounds keep a variable fixed",
    "Town03 junction, clear noon and green lights remain fixed; the scene seed is separate from the policy seed",
    "One model load per batch; reset ego, sensors, histories and controller for each sampled start",
], "Implementation: sample_config() creates a FeatureSpace from Box/Constant domains and calls "
   "FeatureSampler.randomSamplerFor(...).nextSample(). launch() writes an explicit rollout list "
   "to temporary YAML and invokes local/run.sh once; run_intersection.py loads the sharded model "
   "once and loops over those starts. VerifAI dependencies are isolated in .runtime/verifai-venv/; "
   "the working model environment and controller are unchanged. This is passive random sampling, "
   "with no adaptive monitor or additional actors. VerifAI installs Scenic as a package dependency, "
   "but no Scenic scenario scripts are used. Docs: https://verifai.readthedocs.io/en/latest/basic_usage.html .")

append("Launching sampled rollouts: server and controls", [
    "Start/stop CARLA: bash local/server.sh start / stop. Default connection: 127.0.0.1:2000",
    "Launch: bash local/run_verifai.sh. Config: local/configs/verifai.yaml; default batch is 2 rollouts × 8 s",
    "sampling.count sets the count; sampling.seed and ranges define samples. --limit caps count; --duration sets simulated seconds",
    "--sample-only prints replayable YAML. VerifAI runs directly inside the launcher; no separate VerifAI server is started",
], "From the carlamayo repository: bash local/setup_verifai.sh (one-time installation), "
   "bash local/server.sh start, bash local/run_verifai.sh, bash local/server.sh stop. "
   "A short check: bash local/run_verifai.sh --limit 1 --duration 1 --output outputs/verifai-smoke. "
   "--limit cannot increase sampling.count; edit the YAML to increase the number of rollouts. "
   "--host and --port select the CARLA server; --checkpoint selects cached model weights. "
   "The server/client API explored initially is not implemented: this minimal version uses "
   "VerifAI's sampler directly. The CARLA Docker server is a separate process managed by server.sh. "
   "The top-level seed controls existing model sampling; sampling.seed controls initial states.")

append("Verification and GitHub workflow", [
    "16 sampling/handoff tests passed; two fresh 1 s rollouts completed with 10 model calls each and one model load",
    "Both runs saved front/overhead videos, traces and exact samples; replay config.json through local/run.sh",
    "Created abhip02/Carlamayo fork: origin points to the fork; upstream retains aveeslab/Carlamayo",
    "Branch feature/verifai-rollouts; commit 0fa67d6. Push dry run passed; short runs validate integration only",
], "Test command: .runtime/verifai-venv/bin/python -m pytest -q tests/test_verifai_sampling.py "
   "(16 passed). GitHub Actions now has a dedicated sampler-test job; hosted CI has not been run "
   "because local commits remain unpushed. End-to-end command: bash local/run_verifai.sh "
   "--duration 1 --output outputs/verifai-integration-smoke. Samples: "
   "A distance 23.197134 m, speed 3.275029 m/s, heading -4.749892 degrees; "
   "B distance 21.116054 m, speed 3.676699 m/s, heading +2.364712 degrees; both lateral 0. "
   "Scene seed 42, policy seed 42. Both ended at duration_reached with no recorded collision. "
   "Each front/overhead video contains 11 frames at 10 fps (handover plus 10 steps). "
   "Wall times 47.39 s and 34.11 s, including initialization. CARLA was stopped after the test. "
   "Saved samples/version/replay config were verified. These short runs do not establish navigation "
   "success, failure probabilities or adaptive falsification. Fork: https://github.com/abhip02/Carlamayo . "
   "Push command: git push -u origin feature/verifai-rollouts .")

generated = work / "generated.pptx"
prs.save(generated)
changed = {
    "ppt/presentation.xml", "ppt/_rels/presentation.xml.rels", "[Content_Types].xml",
    str(status.part.partname).lstrip("/"),
    str(status.notes_slide.part.partname).lstrip("/"),
}
with zipfile.ZipFile(source) as src, zipfile.ZipFile(generated) as gen:
    original = {n: src.read(n) for n in src.namelist()}
    parts = {n: gen.read(n) for n in gen.namelist()}
    for name, data in original.items():
        if name not in changed:
            parts[name] = data

# python-pptx renumbers slide parts when loading a previously reordered deck.
# Keep the source's existing IDs/targets/order and append only new relationships.
p_ns = "http://schemas.openxmlformats.org/presentationml/2006/main"
r_ns = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
presentation_part = "ppt/presentation.xml"
rels_part = "ppt/_rels/presentation.xml.rels"
source_presentation = etree.fromstring(original[presentation_part])
generated_presentation = etree.fromstring(parts[presentation_part])
source_ids = source_presentation.find(f"{{{p_ns}}}sldIdLst")
new_ids = list(generated_presentation.find(f"{{{p_ns}}}sldIdLst"))[17:]
source_rels = etree.fromstring(original[rels_part])
generated_rels = {r.get("Id"): r for r in etree.fromstring(parts[rels_part])}
old_rel_ids = {r.get("Id") for r in source_rels}
for slide_id in new_ids:
    rid = slide_id.get(f"{{{r_ns}}}id")
    assert rid not in old_rel_ids
    source_ids.append(deepcopy(slide_id))
    source_rels.append(deepcopy(generated_rels[rid]))
parts[presentation_part] = etree.tostring(source_presentation, xml_declaration=True, encoding="UTF-8", standalone=True)
parts[rels_part] = etree.tostring(source_rels, xml_declaration=True, encoding="UTF-8", standalone=True)
source_types = etree.fromstring(original["[Content_Types].xml"])
def type_key(element):
    return element.tag, element.get("PartName"), element.get("Extension")
existing_types = {type_key(e) for e in source_types}
for element in etree.fromstring(parts["[Content_Types].xml"]):
    if type_key(element) not in existing_types:
        source_types.append(deepcopy(element))
parts["[Content_Types].xml"] = etree.tostring(source_types, xml_declaration=True, encoding="UTF-8", standalone=True)

output = work / "abhi_thesis_updates.pptx"
with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as z:
    for name, data in parts.items():
        z.writestr(name, data)
checked = Presentation(output)
assert len(checked.slides) == 20
expected_titles = source_titles[:]
expected_titles[12] = status.shapes.title.text
assert [s.shapes.title.text for s in list(checked.slides)[:17]] == expected_titles
with zipfile.ZipFile(output) as z:
    assert all(z.read(n) == data for n, data in original.items() if n not in changed)
    movies = [n for n in z.namelist() if n.endswith(".mp4")]
    assert len(movies) == 4
    assert all(z.read(n) == original[n] for n in movies)
manifest = {
    "source_slides": 17, "updated_slide": 13, "added_slides": [18, 19, 20],
    "total_slides": 20, "source_parts_changed": sorted(changed),
    "source_slide_order_preserved": True,
    "all_other_source_parts_preserved": True, "embedded_videos_preserved": movies,
    "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
}
(work / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))

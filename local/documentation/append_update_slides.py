"""Append the October 2026 CarlaMayo update, preserving every original slide part.

Run with python-pptx available; pass the documentation output directory containing
original.pptx and the two prepared H.264 clips/poster PNGs. No Drive API writes.
"""
from copy import deepcopy
from pathlib import Path
import json
import sys
import zipfile

from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.oxml.xmlchemy import OxmlElement

WORK = Path(sys.argv[1]).resolve()
SOURCE = WORK / "original.pptx"
FRONT_URL = "https://drive.google.com/file/d/1mL6NOt5Ek9rSOolD030n4WtIMzBdp30-/view"
OVERHEAD_URL = "https://drive.google.com/file/d/10R5qxdlxMedw4lRRgyi7VGED94fDQlEx/view"

prs = Presentation(SOURCE)
assert len(prs.slides) == 9, "Review the source if the deck changed."
exemplar = prs.slides[1]


def paragraphs(shape, texts, size=None, spacing=None, bullet=True):
    body = shape._element.txBody
    templates = [deepcopy(p) for p in body.p_lst]
    for p in list(body.p_lst):
        body.remove(p)
    for i, text in enumerate(texts):
        p = deepcopy(templates[min(i, len(templates)-1)])
        runs = list(p.r_lst)
        assert len(runs) == 1, "Only uniform exemplar paragraphs are replaced."
        runs[0].t.text = text
        if size is not None:
            runs[0].get_or_add_rPr().set("sz", str(int(size*100)))
            for n in p.pPr.findall("{http://schemas.openxmlformats.org/drawingml/2006/main}buSzPts"):
                n.set("val", str(int(size*100)))
        if spacing is not None:
            sp = p.pPr.find("{http://schemas.openxmlformats.org/drawingml/2006/main}spcBef")
            sp[0].set("val", str(0 if i == 0 else int(spacing*100)))
        if not bullet:
            for tag in ("buChar", "buClr", "buSzPts"):
                for n in p.pPr.findall("{http://schemas.openxmlformats.org/drawingml/2006/main}"+tag):
                    p.pPr.remove(n)
            p.pPr.append(OxmlElement("a:buNone"))
            p.pPr.set("marL", "0")
            p.pPr.set("indent", "0")
        body.append(p)


def add_slide(title, items, notes, *, media=None):
    slide = prs.slides.add_slide(exemplar.slide_layout)
    for shape in list(slide.shapes):
        shape._element.getparent().remove(shape._element)
    for shape in exemplar.shapes:
        slide.shapes._spTree.insert_element_before(deepcopy(shape._element), "p:extLst")
    title_shape, body = slide.shapes
    paragraphs(title_shape, [title])
    paragraphs(body, items)
    slide.notes_slide.notes_text_frame.text = notes
    if media:
        stem, url, x, y, w, h, bx, bw = media
        body.left, body.top = Inches(bx), Inches(1.7)
        body.width, body.height = Inches(bw), Inches(4.8)
        paragraphs(body, items, size=20, spacing=18)
        slide.shapes.add_movie(str(WORK/(stem+".mp4")), Inches(x), Inches(y),
                               Inches(w), Inches(h), poster_frame_image=str(WORK/("front-poster.png" if stem.endswith("front") else "overhead-poster.png")), mime_type="video/mp4")
        caption = slide.shapes.add_textbox(Inches(0.8), Inches(6.65), Inches(11.8), Inches(0.4))
        p = caption.text_frame.paragraphs[0]
        r = p.add_run()
        r.text = "Play video in Google Drive"
        r.font.name, r.font.size = "Arial", Pt(16)
        r.hyperlink.address = url
    return slide


add_slide("CarlaMayo: Alpamayo driving in CARLA", [
    "CARLA supplies four camera views (four frames each) and 16 ego-history poses",
    "Alpamayo 1.5 uses images, history and navigation text to predict 64 poses over 6.4 s",
    "CarlaMayo converts the plan to world coordinates; pure-pursuit and speed control produce steering, throttle and brake",
    "Advance CARLA by 0.1 s and replan; text instructions condition the model but do not enforce a route",
], "Local sources: module/adapters/alpamayo_1_5.py; module/open_loop_dataset.py; module/trajectory_follower.py; local/run_intersection.py. Four views: cross-left, front-wide, cross-right, front-tele. Histories and predictions use the model rear-axle frame. The existing follower differs from AlpaSim's MPC. CARLA waits during model inference in synchronous mode. Upstream: https://github.com/aveeslab/Carlamayo ; model: https://github.com/NVlabs/alpamayo1.5 .")

add_slide("Our integration: sharded model + isolated runner", [
    "Separate carlamayo repository and local/ runner; CARLA 0.9.16 renders on GPU 3",
    "Alpamayo runs in BF16/SDPA on GPUs 2 and 1; paired VLM/expert layers split at 0–19 / 20–35",
    "Reuse cached weights and ML packages; load the model once, then reset ego, sensors, history and controller for each rollout",
    "Two 8 s rollouts completed (80 model calls each, about 4–5 min wall time); both deviated from the straight-through instruction",
], "Upstream commit 4914558f868d8e51d482dbcdaf833ad7e2ce7afe; local branch local/sharded-intersection. Integration commit 738dee3; overlay commit 5d677a9. Explicit device map co-locates corresponding VLM and action-expert layers. Existing AlpaSim/Alpamayo repos and base environment were unchanged. Runtime/config/baseline records are in local/. No recorded collisions or OOM in the two runs. Wall times including initialization: 246.90 and 279.72 s. Completion is not a navigation success criterion. Full provenance and rollback: local/README.md and local/PROGRESS.md.")

add_slide("Scene: one fixed Town03 intersection", [
    "Junction 103, road 184, lane −1; clear noon, traffic lights frozen green, no spawned traffic",
    "Define ego by distance before the entry, lateral offset, speed and heading offset; positive offsets point right",
    "Render a constant-speed/heading pre-roll to fill camera and pose histories, then hand control to vehicle physics",
    "Run A: 20 m, 3 m/s, 0°; run B: 25 m, 4 m/s, +5°. Both use: “Continue straight through the intersection.”",
], "Entry x=-6.094448 m, y=118.324844 m, yaw=89.637459 degrees. Position = entry - d*forward + lateral*right; yaw = entry_yaw + heading_offset. Positions refer to the CARLA actor origin. The vehicle settles first, then 16 history samples and four camera frames are populated; the last warm-up tick enables physics and sets the requested velocity. Both demonstration runs have lateral offset 0. Actual handover speeds were 2.9524 and 3.9060 m/s. Actual states are in each initial.json. Static map objects remain. Each rollout ends on collision or the duration limit; infrastructure errors abort the batch. Sources: local/configs/demo.yaml, local/run_intersection.py, outputs/demo-two-starts/batch-summary.json.")

add_slide("Initial states: current grid and probability model", [
    "Implemented today: an explicit list of starts or a Cartesian grid. Random sampling is not implemented yet",
    "Configured grid: distance {10, 20, 30} m × speed {0, 3, 6} m/s × heading {−10°, 0°, +10°}; lateral offset 0",
    "Illustrative probabilistic extension: choose each variable independently and uniformly from those sets (27 equally likely states)",
    "Save each sampled state and seed, then reuse the same pre-roll and simulation loop. Only the two explicit demo starts have been run",
], "This slide separates existing code from a proposed sampling interpretation. local/configs/intersection.yaml defines the full 27-state Cartesian product, which has not been executed. Independent discrete-uniform sampling would assign probability 1/27 to each grid state; it would sample with replacement unless otherwise specified. No distribution or scenario sampler has been implemented. Current seed 42 fixes model sampling; it does not randomize scene initialization. Continuous distributions, Scenic, NPC sampling and constrained rejection sampling are not implemented. No new implementation is requested or performed by this documentation update.")

add_slide("Original rollout: front camera", [
    "Run B: 25 m before entry, 4 m/s, +5° heading",
    "8 s of simulation; 80 model predictions",
    "Instruction: continue straight through the intersection",
    "Completed without a recorded collision; driving behavior still needs investigation",
], "Original source: outputs/demo-two-starts/001_angled/video.mp4. Encoded as H.264 for presentation compatibility without changing the simulation sequence. Front-wide is one of four model camera streams; the displayed recording includes time and speed labels. 81 video frames at 10 fps (initial handover plus 80 steps), hence an 8.1 s video for an 8.0 s rollout. Inference wall time is not played back as pauses. Embedded MP4 is for compatible PowerPoint playback; the Drive link is the browser fallback. " + FRONT_URL,
    media=("carlamayo-angled-front", FRONT_URL, 0.8, 1.7, 8.4, 4.725, 9.5, 3.05))

add_slide("Same rollout: overhead + predicted / actual paths", [
    "Cyan: actual rear-axle trail",
    "Yellow: latest 6.4 s prediction, replaced every 0.1 s",
    "Saved-pose reconstruction of run B; inset is the original front video",
    "Plans change direction despite the straight instruction. Cause not isolated; investigation paused",
], "Source: outputs/demo-two-starts-overhead/001_angled/overhead.mp4; reconstructed from outputs/demo-two-starts/001_angled/ego.csv and predictions.jsonl. Vehicle physics disabled during reconstruction; no new model inference. This is not a second independent rollout or a native CARLA recorder replay. Camera is 80 m above the road and centered 15 m beyond the entry; approach points up. Both paths reference the rear axle; predictions are anchored at inference-time pose and displayed after the next 0.1 s control step. Lines are OpenCV overlays, never model inputs. Live overhead recording was separately verified in a one-second, 10-call sharded run. Position, speed and heading all changed between A and B, so the cause of differing behavior is not isolated. " + OVERHEAD_URL,
    media=("carlamayo-angled-overhead", OVERHEAD_URL, 0.8, 1.65, 6.4, 4.8, 7.65, 4.85))

intermediate = WORK / "generated.pptx"
prs.save(intermediate)
# python-pptx may reserialize source parts. Keep all source bytes except the
# three package indexes which must enumerate the newly appended slides/media.
index_parts = {"ppt/presentation.xml", "ppt/_rels/presentation.xml.rels", "[Content_Types].xml"}
with zipfile.ZipFile(SOURCE) as source, zipfile.ZipFile(intermediate) as generated:
    original = {n: source.read(n) for n in source.namelist()}
    parts = {n: generated.read(n) for n in generated.namelist()}
    for n, data in original.items():
        if n not in index_parts:
            parts[n] = data
target = WORK / "abhi_thesis_updates.pptx"
with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as out:
    for name, data in parts.items():
        out.writestr(name, data)
check = Presentation(target)
assert len(check.slides) == 15
with zipfile.ZipFile(target) as updated:
    assert all(updated.read(n) == data for n, data in original.items() if n not in index_parts)
    media = [n for n in updated.namelist() if n.endswith('.mp4')]
    assert len(media) == 2
manifest = {"source_slides":9,"added_slides":6,"total_slides":15,"original_parts_preserved":True,
            "embedded_videos":media,"front_url":FRONT_URL,"overhead_url":OVERHEAD_URL,
            "output":str(target)}
(WORK / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
print(json.dumps(manifest, indent=2))

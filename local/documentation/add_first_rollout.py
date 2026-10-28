"""Extend the current Drive deck to show both recorded rollouts in both views.

Usage: python add_first_rollout.py OUTPUT_DIRECTORY
The directory contains the downloaded original.pptx and prepared run-A media.
Run-B media is reused from the prior documentation output. No Drive writes.
"""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import sys
import zipfile

from pptx import Presentation
from pptx.enum.shapes import MSO_SHAPE_TYPE

WORK = Path(sys.argv[1]).resolve()
PREVIOUS = WORK.parent / "documentation-20261028"
SOURCE = WORK / "original.pptx"
prs = Presentation(SOURCE)
assert len(prs.slides) == 15, "Review the current deck before extending it."
front_b, overhead_b = prs.slides[13], prs.slides[14]
assert front_b.shapes.title.text == "Original rollout: front camera"
assert overhead_b.shapes.title.text == "Same rollout: overhead + predicted / actual paths"
URLS = {
    "A_front": "https://drive.google.com/file/d/1irYr8jaVTKbys64Gged4A8HpeOkHmjQ3/view",
    "A_overhead": "https://drive.google.com/file/d/1hUe8mfhK59jkom3TQtE1Y750KvQ1wDG_/view",
    "B_front": "https://drive.google.com/file/d/1mL6NOt5Ek9rSOolD030n4WtIMzBdp30-/view",
    "B_overhead": "https://drive.google.com/file/d/10R5qxdlxMedw4lRRgyi7VGED94fDQlEx/view",
}


def duplicate(source):
    slide = prs.slides.add_slide(source.slide_layout)
    for shape in list(slide.shapes):
        shape._element.getparent().remove(shape._element)
    rid_map = {}
    for rel in source.part.rels.values():
        if rel.reltype.endswith(("/slideLayout", "/notesSlide")):
            continue
        target = rel.target_ref if rel.is_external else rel.target_part
        rid_map[rel.rId] = slide.part.relate_to(target, rel.reltype, rel.is_external)
    for shape in source.shapes:
        element = deepcopy(shape._element)
        for node in element.iter():
            for key, value in list(node.attrib.items()):
                if key.startswith("{http://schemas.openxmlformats.org/officeDocument/2006/relationships}"):
                    node.set(key, rid_map[value])
        slide.shapes._spTree.insert_element_before(element, "p:extLst")
    return slide


def replace_text(shape, texts):
    assert len(shape.text_frame.paragraphs) == len(texts)
    for paragraph, text in zip(shape.text_frame.paragraphs, texts):
        assert len(paragraph.runs) == 1
        paragraph.runs[0].text = text


def configure(slide, run, view):
    title = f"Rollout {run}: " + ("original front camera" if view == "front" else "overhead + predicted / actual paths")
    replace_text(slide.shapes.title, [title])
    if run == "A":
        body = slide.shapes[1]
        if view == "front":
            replace_text(body, [
                "Run A: 20 m before entry, 3 m/s, 0° heading",
                "8 s of simulation; 80 model predictions",
                "Instruction: continue straight through the intersection",
                "No recorded collision; the path still deviates from the straight instruction",
            ])
        else:
            replace_text(body, [
                "Cyan: actual rear-axle trail",
                "Yellow: latest 6.4 s prediction, replaced every 0.1 s",
                "Saved-pose reconstruction of run A; inset is the original front video",
                "Starts aligned with the lane, but the executed path curves away from straight",
            ])
        source = "outputs/demo-two-starts/000_straight/"
        notes = f"Rollout A requested 20 m before the entry, 3 m/s and zero heading/lateral offsets. Actual handover speed 2.9524 m/s. 8 simulated seconds, 80 model calls, no recorded collision. Source: {source}video.mp4; ego.csv and predictions.jsonl. Video contains 81 frames at 10 fps, including handover."
        if view == "overhead":
            notes += " Overhead source: outputs/demo-two-starts-overhead/000_straight/overhead.mp4. Saved-pose reconstruction with vehicle physics disabled and no new inference; inset is the original front video. Cyan is the rear-axle trail; yellow is the latest 6.4-second plan anchored at inference time. Overlays are never model inputs."
        slide.notes_slide.notes_text_frame.text = notes + " " + URLS[f"{run}_{view}"]
    caption = next(s for s in slide.shapes if s.has_text_frame and s.text == "Play video in Google Drive")
    caption.text_frame.paragraphs[0].runs[0].hyperlink.address = URLS[f"{run}_{view}"]
    poster = next(s for s in slide.shapes if s.shape_type == MSO_SHAPE_TYPE.PICTURE)
    xywh = (poster.left, poster.top, poster.width, poster.height)
    poster._element.getparent().remove(poster._element)
    if run == "A":
        clip = WORK / f"carlamayo-straight-{view}.mp4"
        image = WORK / f"straight-{view}-poster.png"
    else:
        clip = PREVIOUS / f"carlamayo-angled-{view}.mp4"
        image = PREVIOUS / f"{view}-poster.png"
    slide.shapes.add_movie(str(clip), *xywh, poster_frame_image=str(image), mime_type="video/mp4")


front_a, overhead_a = duplicate(front_b), duplicate(overhead_b)
for slide, run, view in [(front_a,"A","front"),(front_b,"B","front"),
                         (overhead_a,"A","overhead"),(overhead_b,"B","overhead")]:
    configure(slide, run, view)

# Group the original views first, then the overhead views, without changing
# the first thirteen slides or the existing slide identities.
ids = list(prs.slides._sldIdLst)
for node in ids:
    prs.slides._sldIdLst.remove(node)
for node in ids[:13] + [ids[15], ids[13], ids[16], ids[14]]:
    prs.slides._sldIdLst.append(node)

generated = WORK / "generated.pptx"
prs.save(generated)
changed = {"ppt/presentation.xml", "ppt/_rels/presentation.xml.rels", "[Content_Types].xml",
           "ppt/slides/slide14.xml", "ppt/slides/slide15.xml",
           "ppt/slides/_rels/slide14.xml.rels", "ppt/slides/_rels/slide15.xml.rels"}
with zipfile.ZipFile(SOURCE) as original_zip, zipfile.ZipFile(generated) as generated_zip:
    original = {n: original_zip.read(n) for n in original_zip.namelist()}
    parts = {n: generated_zip.read(n) for n in generated_zip.namelist()}
    for name, data in original.items():
        if name not in changed:
            parts[name] = data
output = WORK / "abhi_thesis_updates.pptx"
with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as z:
    for name, data in parts.items():
        z.writestr(name, data)
assert len(Presentation(output).slides) == 17
with zipfile.ZipFile(output) as z:
    assert all(z.read(n) == b for n,b in original.items() if n not in changed)
    movies = [n for n in z.namelist() if n.endswith('.mp4')]
    assert len(movies) == 4
manifest = {"total_slides":17,"unchanged_opening_slides":13,
            "video_order":["A front","B front","A overhead","B overhead"],
            "embedded_videos":movies,"drive_videos":URLS,
            "source_sha256":hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            "output_sha256":hashlib.sha256(output.read_bytes()).hexdigest()}
(WORK / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
print(json.dumps(manifest, indent=2))

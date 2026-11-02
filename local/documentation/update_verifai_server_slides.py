"""Correct the two VerifAI slides in an existing deck; preserve all other parts.

Usage: python update_verifai_server_slides.py OUTPUT_DIRECTORY
The directory must contain the latest Drive PowerPoint as original.pptx.
"""
from pathlib import Path
import hashlib
import json
import posixpath
import sys
import zipfile

from lxml import etree

work = Path(sys.argv[1]).resolve()
source = work / "original.pptx"
ns = {"a": "http://schemas.openxmlformats.org/drawingml/2006/main",
      "p": "http://schemas.openxmlformats.org/presentationml/2006/main"}
updates = {
    "VerifAI: sample initial states, then launch rollouts": (
        ["VerifAI server launches the simulations",
         "VerifAI’s server samples an initial state and triggers each simulation through a persistent CarlaMayo client",
         "The client resets ego, sensors and history, runs the CARLA rollout, and returns its summary; Alpamayo stays loaded",
         "The server scores the result and feeds it back to the sampler before requesting the next rollout",
         "Current monitor: +1 for no recorded collision, −1 for collision; this does not check route completion"],
        "Updated 2026-11-01 from local/verifai_server.py, local/verifai_client.py and local/README.md. "
        "RolloutServer subclasses verifai.server.Server. The launcher starts one persistent client subprocess; "
        "the inherited Server.run_server() loop samples, requests an episode, evaluates the returned summary "
        "and supplies monitor feedback to the sampler. JSON sockets keep VerifAI dependencies in the sampler "
        "environment while the client retains the sharded Alpamayo model in the model environment. "
        "The CARLA simulator process is started separately. The collision-only monitor returns +1 for "
        "duration_reached with no recorded collision and -1 for a collision. Infrastructure/model/client "
        "errors abort rather than receiving a safety score. Random sampling is the default; Halton and "
        "Cross-Entropy are available. Cross-Entropy can adapt using negative scores. No Scenic scripts "
        "or actor-generation extension is used. The original passive batch sampler remains available as "
        "local/run_verifai.sh, but it is not the server/client launcher described here."),
    "Launching sampled rollouts: server and controls": (
        ["Launching through VerifAI: server and controls",
         "Start CARLA separately: bash local/server.sh start; stop with local/server.sh stop. CARLA listens on 127.0.0.1:2000",
         "Launch: bash local/run_verifai_server.sh starts the VerifAI server and client; default is 2 rollouts × 8 s",
         "local/configs/verifai.yaml sets count, seed and ranges. --limit caps count; --duration sets simulated seconds",
         "--sampler selects random, halton or ce; --verifai-port selects the coordinator port (default 8888)"],
        "From the carlamayo repository: bash local/setup_verifai.sh (one-time installation), "
        "bash local/server.sh start, bash local/run_verifai_server.sh, bash local/server.sh stop. "
        "Short check: bash local/run_verifai_server.sh --limit 2 --duration 1 --output outputs/verifai-server-demo. "
        "The launcher starts both the actual VerifAI Server subclass and one CarlaMayo client automatically. "
        "It does not start the CARLA Docker process. sampling.count sets episode count; sampling.seed and "
        "sampling.ranges set initial-state sampling; the top-level seed controls policy sampling. "
        "--host and --port select CARLA; --verifai-port selects the independent local coordinator port "
        "(8888 by default, 0 selects a free port). --sampler random is uniform, halton and ce select Halton "
        "and Cross-Entropy. --sample-only belongs to the older passive launcher, not this server launcher. "
        "Completed samples, scores and summaries are saved in verifai-results.jsonl, with exact states in "
        "config.json for replay through local/run.sh. This wording update ran no new simulations."),
}

with zipfile.ZipFile(source) as z:
    parts = {n: z.read(n) for n in z.namelist()}
original = dict(parts)
changed = []
matched = set()
for name, data in original.items():
    if not name.startswith("ppt/slides/slide") or not name.endswith(".xml"):
        continue
    root = etree.fromstring(data)
    text = root.findall(".//a:t", ns)
    if not text or text[0].text not in updates:
        continue
    title = text[0].text
    assert title not in matched
    matched.add(title)
    replacement, notes = updates[title]
    assert len(text) == len(replacement), "Review mixed-style content before replacing text."
    for node, value in zip(text, replacement):
        node.text = value
    parts[name] = etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)
    changed.append(name)
    rel_name = posixpath.join(posixpath.dirname(name), "_rels", posixpath.basename(name)+".rels")
    rels = etree.fromstring(parts[rel_name])
    note_rel = next(r for r in rels if r.get("Type").endswith("/notesSlide"))
    note_name = posixpath.normpath(posixpath.join(posixpath.dirname(name), note_rel.get("Target")))
    note_root = etree.fromstring(parts[note_name])
    body = note_root.xpath(".//p:sp[p:nvSpPr/p:nvPr/p:ph[@type='body']]", namespaces=ns)
    note_text = body[0].findall(".//a:t", ns)
    assert len(note_text) == 1, "Review note formatting before replacing text."
    note_text[0].text = notes
    parts[note_name] = etree.tostring(note_root, xml_declaration=True, encoding="UTF-8", standalone=True)
    changed.append(note_name)
assert matched == set(updates)
output = work / "abhi_thesis_updates.pptx"
with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as z:
    for name, data in parts.items():
        z.writestr(name, data)
assert all(parts[n] == data for n,data in original.items() if n not in changed)
manifest = {"changed_parts": changed, "preserved_all_other_parts": True,
            "embedded_videos_preserved": [n for n in parts if n.endswith('.mp4')],
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest()}
(work / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
print(json.dumps(manifest, indent=2))

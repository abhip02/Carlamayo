# Presentation update

The 2026-10-28 update appends six slides to the existing nine-slide PowerPoint.
`slide-plan.json` records the source exemplar and media mapping.
`append_update_slides.py` builds the update with python-pptx while retaining every
original ZIP part byte-for-byte except the three package indexes that enumerate
the added slides and assets.

Generated assets, source backup, final deck, Drive readback, PDF previews and
rendered slides are in `../../outputs/documentation-20261028/` (ignored by Git).
The authoring dependencies are isolated under `../../.runtime/docsdeps/`.
No simulation dependencies or model/controller code changed.

To rebuild from the prepared assets, from the repository root:

```bash
PYTHONPATH=.runtime/docsdeps .venv/bin/python \
  local/documentation/append_update_slides.py outputs/documentation-20261028
```

The Drive file ID remains `1yKSWESk_3YMKYPaRN6RDF43gdp5yf-PY`; its name, folder
and sharing were preserved. To revert the deck, replace that file's raw bytes
with `outputs/documentation-20261028/original.pptx` using Drive's file-update
operation. Do not delete the existing file or create a replacement ID.

- Original SHA-256: `7f2d2a25aad94317228b443729babb0d5a61b3254e474d68f7e05242b72f55dc`
- Updated SHA-256: `35c86524473f9f6907a030f6002a3d9da9316965a85ea8cfb28b9ed73137098d`
- Drive readback matched the reviewed output byte-for-byte.
- Temporary native Slides preview was used for rendering; the separate existing
  native `abhi_thesis_updates` copy was not modified.

## Follow-up: both rollouts

The follow-up deck has 17 slides. Slides 14–15 show the original front views of
rollouts A/B; slides 16–17 show their respective overhead reconstructions.
`both-rollouts-slide-plan.json` records how the existing video slides were reused.
The first 13 slides are preserved byte-for-byte. Run B's two existing slides are
relabeled and kept; two run-A slides are added and the video slides are reordered.

All four video files are embedded in the updated PowerPoint. Drive playback links
provide a browser alternative; sharing on the new assets was not broadened.
The source downloaded from Drive for this follow-up contained only posters and
links, so the run-B embedded movie data was restored from the original clips.
Both overhead clips are labeled as reconstructions from saved poses/predictions.
No new inference was run.

Rebuild the follow-up using its prepared assets:

```bash
PYTHONPATH=.runtime/docsdeps .venv/bin/python \
  local/documentation/add_first_rollout.py outputs/documentation-both-rollouts-20261028
```

The follow-up's source backup, final deck, manifest and renders live in
`outputs/documentation-both-rollouts-20261028/`. To undo only this follow-up,
restore that directory's `original.pptx` to the same Drive file ID.

- Follow-up source SHA-256: `2f970085a6abc44fb29e2b562e8a98c0ed02ca1c4abfd7c539a745d916702e97`
- Follow-up output SHA-256: `47ba3830b7f201f51a61e7f78938a26511d66e0f0ba4339cebe10ddd94235f2e`

## Follow-up: VerifAI integration

The next update has 20 slides. Slide 13 now describes the implemented sampling
extension, replacing the obsolete statement that random sampling is unimplemented.
Slides 18–20 summarize the direct VerifAI sampler, CARLA server/rollout controls,
the 16 passing tests and two one-second integration rollouts, and the GitHub fork
and feature branch. The eight-second default is distinguished from the one-second
checks actually executed. No new inference was run for this presentation update.

`verifai-slide-plan.json` records the source, exemplars and media preservation.
`add_verifai_update.py` duplicates the existing text-slide style and preserves the
original slide order and every source ZIP part except the package indexes and
slide 13's text/notes. All four embedded videos and their links remain unchanged.

Rebuild from the downloaded backup:

```bash
PYTHONPATH=.runtime/docsdeps .venv/bin/python \
  local/documentation/add_verifai_update.py outputs/documentation-verifai-20261028
```

The backup, final deck, manifest, complete native preview readbacks and reviewed
PDF/PNGs live in `outputs/documentation-verifai-20261028/` (ignored by Git).
The existing Drive file was updated in place; its ID, name and folder are unchanged.
The uploaded file was downloaded and matched the reviewed output byte-for-byte.
Both temporary native preview decks were deleted after verification. The separate
existing native Slides copy was not changed.

- Source SHA-256: `47ba3830b7f201f51a61e7f78938a26511d66e0f0ba4339cebe10ddd94235f2e`
- Updated SHA-256: `6d23030c82a9158774f75f822086849d65ae720bf4aba67de767d3fccfb9ed18`
- To undo this update, restore `outputs/documentation-verifai-20261028/original.pptx`
  to Drive file `1yKSWESk_3YMKYPaRN6RDF43gdp5yf-PY` with a raw-byte update.

## Server wording — 2026-11-01

Updated the two VerifAI overview/control slides (17–18 in the current 19-slide
deck) to describe the actual server/client implementation: the VerifAI server
samples and triggers episodes through the persistent CarlaMayo client, evaluates
the returned summaries, and supplies sampler feedback. The launch command is now
`bash local/run_verifai_server.sh`; CARLA itself is still started separately.
Removed the obsolete statement that no VerifAI server is started.

`update_verifai_server_slides.py` changes only these slides' text and notes.
All other ZIP parts, including the four videos and the user's recent slide edits,
are preserved. The latest source backup, updated deck, manifest and rendered
preview are in `outputs/documentation-verifai-server-20261101/`. Restore its
`original.pptx` to the same Drive ID to undo just this wording update.

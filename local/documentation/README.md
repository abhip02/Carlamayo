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

Both video files are embedded in the PowerPoint. Drive playback links provide
a browser alternative; Drive sharing on those new assets was not broadened.
Video slides use the same second rollout; the overhead is explicitly labeled as
a reconstruction from saved poses and predictions. No new inference was run.

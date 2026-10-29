"""CPU-only comparison of native and adapter preprocessing on a saved input."""
import argparse
import json
from pathlib import Path

import torch

from alpamayo1_5 import helper
from alpamayo1_5.config import Alpamayo1_5Config
from local.experiments.camera_projection import project_history
from module.adapters.alpamayo_1_5 import VLM_IMAGE_PIXELS
from module.alpamayo_compat import patch_legacy_hydra_targets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--projection", default="equidistant")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    patch_legacy_hydra_targets()
    config = Alpamayo1_5Config.from_pretrained(args.checkpoint)
    # Construct the same tokenizer, including trajectory tokens, without weights.
    processor = helper.get_processor(config._build_processor().tokenizer)
    saved = torch.load(args.input, map_location="cpu", weights_only=True)
    data = project_history(saved["data"], args.projection)
    messages = helper.create_message(
        data["image_frames"].flatten(0, 1), camera_indices=data["camera_indices"],
        nav_text=saved["instruction"])
    kwargs = dict(tokenize=True, add_generation_prompt=False, continue_final_message=True,
                  return_dict=True, return_tensors="pt")
    native = processor.apply_chat_template(messages, **kwargs)
    adapter = processor.apply_chat_template(
        messages, min_pixels=VLM_IMAGE_PIXELS, max_pixels=VLM_IMAGE_PIXELS, **kwargs)
    result = {
        "input": str(args.input), "projection": args.projection,
        "processor_class": type(processor.image_processor).__name__,
        "native_grid": native["image_grid_thw"].tolist(),
        "adapter_grid": adapter["image_grid_thw"].tolist(),
        "keys_equal": list(native) == list(adapter),
        "tensors": {key: {"native_shape": list(native[key].shape),
                           "adapter_shape": list(adapter[key].shape),
                           "exact_equal": torch.equal(native[key], adapter[key])}
                    for key in native},
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

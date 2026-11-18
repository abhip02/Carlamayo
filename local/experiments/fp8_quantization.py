"""E4M3 quantize/dequantize accuracy experiments on GPUs without FP8 GEMM.

Weights are rounded once and retained in their original dtype. Activations are
rounded dynamically before each Linear call. This deliberately measures neither
FP8 storage savings nor native FP8 throughput/accumulation behavior.
"""
from collections import Counter

import torch


PRECISIONS = ("bf16", "fp8-w8a8-emulated", "fp8-w8a16-emulated")


def fp8_qdq(value):
    """Symmetric E4M3FN, absmax scale per last-dimension row, FP32 scaling."""
    if value.numel() == 0:
        return value
    work = value.float()
    scale = work.abs().amax(dim=-1, keepdim=True) / 448.0
    # Zero rows stay zero; tiny nonzero rows must not divide by an underflowed scale.
    scale = scale.clamp_min(torch.finfo(torch.float32).tiny)
    quantized = (work / scale).clamp(-448, 448).to(torch.float8_e4m3fn)
    return (quantized.float() * scale).to(value.dtype)


def _activation_hook(module, args, kwargs):
    if args:
        return (fp8_qdq(args[0]), *args[1:]), kwargs
    return args, {**kwargs, "input": fp8_qdq(kwargs["input"])}


@torch.no_grad()
def apply_precision(model, precision):
    """Apply once to an inference model; report exact coverage and exclusions."""
    if precision not in PRECISIONS:
        raise ValueError(f"Unknown precision: {precision}")
    if hasattr(model, "_fp8_experiment_report"):
        raise ValueError("Precision has already been applied to this model")
    dtypes = Counter()
    for parameter in model.parameters():
        dtypes[str(parameter.dtype)] += parameter.numel()
    report = dict(precision=precision, parameter_dtypes_before=dict(dtypes),
                  quantized_layers=[], excluded_layers=[], quantized_weight_elements=0,
                  total_parameter_elements=sum(p.numel() for p in model.parameters()),
                  native_fp8_compute=False, compressed_weight_storage=False,
                  torch_version=torch.__version__, cuda_version=torch.version.cuda,
                  gpus=[dict(index=i, name=torch.cuda.get_device_name(i),
                             capability=list(torch.cuda.get_device_capability(i)))
                        for i in range(torch.cuda.device_count())])
    if precision == "bf16":
        return report
    # Preserve tied embeddings and the vocabulary output head. Norms, convolutions,
    # softmax, attention matmuls, KV cache and trajectory decoding retain base dtypes.
    embedding_weights = {id(m.weight) for m in model.modules()
                         if isinstance(m, torch.nn.Embedding)}
    layers = []
    for name, module in model.named_modules():
        if not isinstance(module, torch.nn.Linear):
            continue
        if name.split(".")[-1] == "lm_head" or id(module.weight) in embedding_weights:
            report["excluded_layers"].append(name)
            continue
        if module.weight.device.type == "meta":
            raise ValueError(f"Cannot quantize offloaded/meta layer {name}")
        if not torch.isfinite(module.weight).all():
            raise ValueError(f"Non-finite weights in {name}")
        layers.append((name, module))
    if not layers:
        raise ValueError("No eligible Linear layers found")
    seen = set()
    for name, module in layers:
        if id(module.weight) not in seen:
            # Chunking bounds temporary allocations for large feed-forward matrices.
            for chunk in module.weight.split(256, dim=0):
                chunk.copy_(fp8_qdq(chunk))
            seen.add(id(module.weight))
            report["quantized_weight_elements"] += module.weight.numel()
        if precision == "fp8-w8a8-emulated":
            module.register_forward_pre_hook(_activation_hook, with_kwargs=True)
        report["quantized_layers"].append(name)
    report.update(format="torch.float8_e4m3fn", weight_scaling="per_output_channel_absmax",
                  activation_scaling=("dynamic_per_token_absmax" if "w8a8" in precision else "none"),
                  scale_dtype="float32", dequantized_storage="original_parameter_dtype",
                  matrix_multiplication="original_BF16_autocast",
                  scope="Linear layers in vision, language, diffusion expert and action projections; excludes lm_head and tied embeddings")
    model._fp8_experiment_report = report
    return report

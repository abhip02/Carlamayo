import copy

import pytest
import torch

from local.experiments.fp8_quantization import apply_precision, fp8_qdq


@pytest.mark.parametrize("device", ["cpu", pytest.param("cuda", marks=pytest.mark.skipif(
    not torch.cuda.is_available(), reason="CUDA unavailable"))])
def test_e4m3_rounding_and_independent_row_scales(device):
    # Known E4M3 grid near 1: 1, 1.125, 1.25. Maximum 448 fixes scale to 1.
    row = torch.tensor([448., 1.06, 1.07, -1.07, 0.], device=device)
    values = torch.stack((row, row * 2, row * 0))
    expected = torch.tensor([[448., 1., 1.125, -1.125, 0.],
                             [896., 2., 2.25, -2.25, 0.], [0.] * 5], device=device)
    torch.testing.assert_close(fp8_qdq(values), expected, rtol=0, atol=0)
    torch.testing.assert_close(values[0], row, rtol=0, atol=0)


@pytest.mark.parametrize("dtype", [torch.float32, torch.bfloat16, torch.float16])
def test_zero_and_small_rows_are_finite_and_preserve_dtype(dtype):
    values = torch.tensor([[0., 0.], [1.e-5, -2.e-5]], dtype=dtype)
    result = fp8_qdq(values)
    assert result.dtype == dtype and torch.isfinite(result).all()
    assert torch.equal(result[0], values[0])
    torch.testing.assert_close(result[1], values[1], rtol=.07, atol=1.e-7)


class TinyModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.embed = torch.nn.Embedding(5, 5)
        self.linear = torch.nn.Linear(5, 2)
        self.lm_head = torch.nn.Linear(5, 5, bias=False)
        self.lm_head.weight = self.embed.weight
        with torch.no_grad():
            self.linear.weight.copy_(torch.tensor([[448., 1.07, 0., 0., 0.],
                                                  [0., 448., 1.07, 0., 0.]]))


@pytest.mark.parametrize("precision", ["fp8-w8a8-emulated", "fp8-w8a16-emulated"])
def test_weight_rounding_activation_hooks_and_tied_head_exclusion(precision):
    model = TinyModel().eval()
    embed, bias = model.embed.weight.clone(), model.linear.bias.clone()
    report = apply_precision(model, precision)
    assert report["quantized_layers"] == ["linear"]
    assert report["excluded_layers"] == ["lm_head"]
    assert report["quantized_weight_elements"] == 10
    assert torch.equal(model.embed.weight, embed)
    assert model.lm_head.weight is model.embed.weight
    assert torch.equal(model.linear.bias, bias)
    assert model.linear.weight[0, 1].item() == 1.125
    x = torch.tensor([[448., 1.07, 1.07, 0., 0.]])
    expected_x = x.clone()
    if "w8a8" in precision:
        expected_x[0, 1:3] = 1.125
    expected = torch.nn.functional.linear(expected_x, model.linear.weight, bias)
    torch.testing.assert_close(model.linear(x), expected, rtol=0, atol=0)
    torch.testing.assert_close(model.linear(input=x), expected, rtol=0, atol=0)
    assert x[0, 1].item() == pytest.approx(1.07)
    with pytest.raises(ValueError, match="already"):
        apply_precision(model, precision)


def test_bf16_is_noop_and_invalid_precision_fails_before_mutation():
    model = TinyModel().eval()
    before = copy.deepcopy(model.state_dict())
    assert apply_precision(model, "bf16")["quantized_layers"] == []
    with pytest.raises(ValueError, match="Unknown"):
        apply_precision(model, "fp8")
    for key, value in model.state_dict().items():
        assert torch.equal(value, before[key])


def test_nonfinite_weights_fail_before_mutation():
    model = TinyModel().eval()
    with torch.no_grad():
        model.linear.weight[0, 0] = float("nan")
    with pytest.raises(ValueError, match="Non-finite"):
        apply_precision(model, "fp8-w8a8-emulated")

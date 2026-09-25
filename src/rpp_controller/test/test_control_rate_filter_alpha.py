"""Per-cycle filter alphas keep their 20 Hz time constant at any control rate."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

RPP = (
    Path(__file__).resolve().parents[1]
    / "rpp_controller"
    / "rpp_controller_node.py"
)


def _class():
    return next(
        n for n in ast.parse(RPP.read_text()).body
        if isinstance(n, ast.ClassDef) and n.name == "RPPController"
    )


def _scaler():
    method = next(
        n for n in _class().body
        if isinstance(n, ast.FunctionDef) and n.name == "rate_scaled_filter_alpha"
    )
    env = {"math": math}
    module = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "<rpp>", "exec"), env)
    return env["rate_scaled_filter_alpha"]


def _reference_hz():
    for node in _class().body:
        if (
            isinstance(node, ast.Assign)
            and node.targets[0].id == "FILTER_ALPHA_REFERENCE_HZ"
        ):
            return node.value.value
    raise AssertionError("FILTER_ALPHA_REFERENCE_HZ missing")


def _scaled(rate_hz, alpha):
    node = NS(CONTROL_HZ=rate_hz, FILTER_ALPHA_REFERENCE_HZ=_reference_hz())
    return _scaler()(node, alpha)


def test_reference_rate_is_the_20_hz_tuning_rate():
    assert _reference_hz() == 20.0


def test_both_filter_alphas_are_rate_scaled():
    source = RPP.read_text()
    for name in ("xtrack_rate_filter_alpha", "moving_yaw_rate_filter_alpha"):
        assignment = source[source.index(f"self.{name} = "):][:120]
        assert "self.rate_scaled_filter_alpha(" in assignment


def _decay_per_second(rate_hz, alpha):
    return (1.0 - alpha) ** rate_hz


def test_reference_rate_is_unchanged():
    assert _scaled(20.0, 0.20) == pytest.approx(0.20)


@pytest.mark.parametrize("alpha", [0.05, 0.20, 0.50, 0.90])
@pytest.mark.parametrize("rate_hz", [10.0, 50.0, 100.0])
def test_time_constant_is_preserved(rate_hz, alpha):
    scaled = _scaled(rate_hz, alpha)
    assert 0.0 < scaled < 1.0
    assert _decay_per_second(rate_hz, scaled) == pytest.approx(
        _decay_per_second(20.0, alpha)
    )


def test_50_hz_alpha_is_smaller_per_cycle():
    assert _scaled(50.0, 0.20) == pytest.approx(1.0 - 0.8 ** 0.4)
    assert _scaled(50.0, 0.20) < 0.20


@pytest.mark.parametrize("alpha", [0.0, 1.0, -0.1, 1.5, math.nan])
def test_boundary_and_invalid_values_pass_through_for_validation(alpha):
    out = _scaled(50.0, alpha)
    if math.isnan(alpha):
        assert math.isnan(out)
    else:
        assert out == alpha

"""Tests for arbitary-mass profile interpolation."""
import pytest
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from powder_sampling_control.dispensing_algorithm.core import (
    DISPENSE_TARGET_PROFILES,
    interpolate_profile,
    validate_dispense_target,
)

# ── anchor points: exact matches ──

def test_interpolate_exact_100():
    profile = interpolate_profile(100)
    for key in DISPENSE_TARGET_PROFILES[100]:
        assert profile[key] == pytest.approx(DISPENSE_TARGET_PROFILES[100][key])

def test_interpolate_exact_300():
    profile = interpolate_profile(300)
    for key in DISPENSE_TARGET_PROFILES[300]:
        assert profile[key] == pytest.approx(DISPENSE_TARGET_PROFILES[300][key])

def test_interpolate_exact_500():
    profile = interpolate_profile(500)
    for key in DISPENSE_TARGET_PROFILES[500]:
        assert profile[key] == pytest.approx(DISPENSE_TARGET_PROFILES[500][key])

# ── interpolation between anchors ──

def test_interpolate_200_midpoint():
    """200 is halfway between 100 and 300."""
    profile = interpolate_profile(200)
    p100 = DISPENSE_TARGET_PROFILES[100]
    p300 = DISPENSE_TARGET_PROFILES[300]
    for key in ("coarse_rate_mg_s", "fine_rate_mg_s", "precision_rate_mg_s",
                "precision_start_remaining_mg", "tail_taper_start_remaining_mg",
                "tail_taper_end_remaining_mg"):
        expected = p100[key] + (p300[key] - p100[key]) * (200 - 100) / (300 - 100)
        assert profile[key] == pytest.approx(expected)

def test_interpolate_400_between_300_500():
    profile = interpolate_profile(400)
    p300 = DISPENSE_TARGET_PROFILES[300]
    p500 = DISPENSE_TARGET_PROFILES[500]
    for key in ("coarse_rate_mg_s", "fine_rate_mg_s", "precision_rate_mg_s",
                "precision_start_remaining_mg", "tail_taper_start_remaining_mg",
                "tail_taper_end_remaining_mg"):
        expected = p300[key] + (p500[key] - p300[key]) * (400 - 300) / (500 - 300)
        assert profile[key] == pytest.approx(expected)

# ── extrapolation below 100 ──

# (50 is no longer a valid target; minimum was raised to 100)

# ── extrapolation above 500 ──

def test_interpolate_750_above_500():
    """Extrapolate upward using 300→500 slope, clamped by coarse upper bound."""
    profile = interpolate_profile(750)
    p300 = DISPENSE_TARGET_PROFILES[300]
    p500 = DISPENSE_TARGET_PROFILES[500]
    slope = (p500["coarse_rate_mg_s"] - p300["coarse_rate_mg_s"]) / (500 - 300)
    raw_coarse = p500["coarse_rate_mg_s"] + slope * (750 - 500)
    # raw extrapolation (37.5) exceeds the 35.0 clamp
    assert raw_coarse > 35.0
    assert profile["coarse_rate_mg_s"] == pytest.approx(35.0)

# ── boundary values ──

def test_interpolate_100_minimum():
    profile = interpolate_profile(100)
    assert profile["coarse_rate_mg_s"] >= 5.0
    assert profile["fine_rate_mg_s"] >= 8.0
    assert profile["precision_rate_mg_s"] >= 5.0

def test_interpolate_1000_maximum():
    profile = interpolate_profile(1000)
    assert profile["coarse_rate_mg_s"] <= 35.0
    assert profile["fine_rate_mg_s"] <= 30.0
    assert profile["precision_rate_mg_s"] <= 20.0

# ── clamp guard: tail_taper_end < tail_taper_start ──

def test_tail_taper_constraint():
    """If interpolation makes tail_taper_end >= tail_taper_start, clamp it."""
    profile = interpolate_profile(100)
    assert profile["tail_taper_end_remaining_mg"] < profile["tail_taper_start_remaining_mg"]

# ── validate_dispense_target ──

def test_validate_valid_targets():
    validate_dispense_target(100)
    validate_dispense_target(500)
    validate_dispense_target(1000)

def test_validate_below_range():
    with pytest.raises(ValueError, match="100"):
        validate_dispense_target(99)

def test_validate_above_range():
    with pytest.raises(ValueError, match="1000"):
        validate_dispense_target(1001)

def test_validate_not_integer():
    with pytest.raises(ValueError):
        validate_dispense_target("abc")

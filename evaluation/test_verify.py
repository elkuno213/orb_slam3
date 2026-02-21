"""Unit tests for the verify.py evaluation helpers.

Tests cover the to_unified() function that transforms TestConfig instances
to use the unified orb_slam3_offline binary.
"""

from __future__ import annotations

import pytest

from verify import (
    UNIFIED_BINARY,
    TestConfig,
    _UNIFIED_MAPPING,
    to_unified,
)


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────


def _make_config(
    name: str,
    binary: str = "legacy_bin",
    command_args: list[str] | None = None,
) -> TestConfig:
    """Create a minimal TestConfig for testing."""
    return TestConfig(
        name=name,
        binary=binary,
        output_format="tum",
        gt_path="ground_truth.txt",
        gt_format="tum",
        alignment="se3",
        settings_file="settings.yaml",
        command_args=command_args or [],
    )


# ─────────────────────────────────────────────────────────────────────────────
# to_unified: basic transformation
# ─────────────────────────────────────────────────────────────────────────────


class TestToUnified:
    """Tests for the to_unified() function."""

    def test_binary_replaced(self) -> None:
        config = _make_config("rgbd_tum_fr1_room", binary="rgbd_tum")
        result = to_unified(config)
        assert result.binary == UNIFIED_BINARY

    def test_original_args_preserved(self) -> None:
        original_args = ["--sequence-dir", "/data/tum/fr1", "--association-file", "assoc.txt"]
        config = _make_config("rgbd_tum_fr1_room", command_args=original_args)
        result = to_unified(config)
        # Original args should appear after the dataset/sensor flags.
        for arg in original_args:
            assert arg in result.command_args

    def test_dataset_sensor_prepended(self) -> None:
        config = _make_config("rgbd_tum_fr1_room")
        result = to_unified(config)
        assert result.command_args[0] == "--dataset"
        assert result.command_args[1] == "tum"
        assert result.command_args[2] == "--sensor"
        assert result.command_args[3] == "rgbd"

    def test_inertial_flag_added(self) -> None:
        config = _make_config("mono_inertial_euroc_v1_01")
        result = to_unified(config)
        assert "--inertial" in result.command_args

    def test_non_inertial_no_flag(self) -> None:
        config = _make_config("stereo_euroc_v1_01")
        result = to_unified(config)
        assert "--inertial" not in result.command_args

    def test_name_preserved(self) -> None:
        config = _make_config("mono_tum_fr1_room")
        result = to_unified(config)
        assert result.name == "mono_tum_fr1_room"

    def test_non_command_fields_preserved(self) -> None:
        config = TestConfig(
            name="rgbd_tum_fr1_room",
            binary="rgbd_tum",
            output_format="tum",
            gt_path="gt.txt",
            gt_format="tum",
            alignment="se3",
            settings_file="special.yaml",
            trajectory_filename="Custom.txt",
        )
        result = to_unified(config)
        assert result.output_format == "tum"
        assert result.gt_path == "gt.txt"
        assert result.gt_format == "tum"
        assert result.alignment == "se3"
        assert result.settings_file == "special.yaml"
        assert result.trajectory_filename == "Custom.txt"


# ─────────────────────────────────────────────────────────────────────────────
# to_unified: all mapped tests produce valid configs
# ─────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("test_name", list(_UNIFIED_MAPPING.keys()))
def test_all_mappings_produce_valid_config(test_name: str) -> None:
    """Every entry in _UNIFIED_MAPPING should produce a valid transformed config."""
    config = _make_config(test_name)
    result = to_unified(config)
    assert result.binary == UNIFIED_BINARY
    assert "--dataset" in result.command_args
    assert "--sensor" in result.command_args


# ─────────────────────────────────────────────────────────────────────────────
# to_unified: error handling
# ─────────────────────────────────────────────────────────────────────────────


def test_unmapped_test_raises_key_error() -> None:
    config = _make_config("nonexistent_test_name")
    with pytest.raises(KeyError):
        to_unified(config)


# ─────────────────────────────────────────────────────────────────────────────
# _UNIFIED_MAPPING: correctness
# ─────────────────────────────────────────────────────────────────────────────


class TestUnifiedMapping:
    """Verify the _UNIFIED_MAPPING dict has correct entries."""

    def test_euroc_entries_are_euroc(self) -> None:
        for name, (dataset, _, _) in _UNIFIED_MAPPING.items():
            if "euroc" in name and "tum" not in name:
                assert dataset == "euroc", f"{name} should map to euroc"

    def test_tum_entries_are_tum(self) -> None:
        for name, (dataset, _, _) in _UNIFIED_MAPPING.items():
            if "tum_fr" in name:
                assert dataset == "tum", f"{name} should map to tum"

    def test_tumvi_entries_are_tumvi(self) -> None:
        for name, (dataset, _, _) in _UNIFIED_MAPPING.items():
            if "tum_vi" in name:
                assert dataset == "tumvi", f"{name} should map to tumvi"

    def test_inertial_entries_have_inertial_true(self) -> None:
        for name, (_, _, inertial) in _UNIFIED_MAPPING.items():
            if "inertial" in name:
                assert inertial is True, f"{name} should have inertial=True"

    def test_non_inertial_entries_have_inertial_false(self) -> None:
        for name, (_, _, inertial) in _UNIFIED_MAPPING.items():
            if "inertial" not in name:
                assert inertial is False, f"{name} should have inertial=False"

    def test_rgbd_entries_have_rgbd_sensor(self) -> None:
        for name, (_, sensor, _) in _UNIFIED_MAPPING.items():
            if name.startswith("rgbd_"):
                assert sensor == "rgbd", f"{name} should map to rgbd sensor"

    def test_stereo_entries_have_stereo_sensor(self) -> None:
        for name, (_, sensor, _) in _UNIFIED_MAPPING.items():
            if name.startswith("stereo_"):
                assert sensor == "stereo", f"{name} should map to stereo sensor"

    def test_mono_entries_have_mono_sensor(self) -> None:
        for name, (_, sensor, _) in _UNIFIED_MAPPING.items():
            if name.startswith("mono_") and not name.startswith("mono_inertial"):
                assert sensor == "mono", f"{name} should map to mono sensor"

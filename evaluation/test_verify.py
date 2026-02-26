"""Unit tests for the verify.py evaluation helpers.

Tests cover build_command(), TestConfig validity, read_euroc_output(),
and compute_ate_rmse().
"""

from __future__ import annotations

import copy

import numpy as np
import pytest
from evo.core.trajectory import PoseTrajectory3D

from verify import (
    BINARY,
    TEST_CONFIGS,
    TestConfig,
    build_command,
    compute_ate_rmse,
    read_euroc_output,
)


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Helpers                                                                                          #


def _make_config(
    name: str = "test_config",
    dataset: str = "euroc",
    sensor: str = "stereo",
    inertial: bool = False,
    output_format: str = "tum",
    command_args: list[str] | None = None,
) -> TestConfig:
    """Create a minimal TestConfig for testing."""
    return TestConfig(
        name=name,
        dataset=dataset,
        sensor=sensor,
        inertial=inertial,
        output_format=output_format,
        gt_path="ground_truth.txt",
        gt_format="tum",
        alignment="se3",
        settings_file="settings.yaml",
        command_args=command_args or [],
    )


def _make_trajectory(positions, timestamps=None):
    """Build a PoseTrajectory3D with identity orientations."""
    n = len(positions)
    if timestamps is None:
        timestamps = np.arange(n, dtype=float)
    quats = np.array([[1, 0, 0, 0]] * n, dtype=float)
    return PoseTrajectory3D(
        positions_xyz=np.asarray(positions, dtype=float),
        orientations_quat_wxyz=quats,
        timestamps=np.asarray(timestamps, dtype=float),
    )


def _write_euroc(tmp_path, data):
    """Write trajectory data to file and return read_euroc_output() result."""
    filepath = tmp_path / "traj.txt"
    np.savetxt(str(filepath), data)
    return read_euroc_output(filepath)


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# build_command: CLI construction                                                                   #


class TestBuildCommand:
    """Tests for the build_command() function."""

    def test_binary_is_first_element(self) -> None:
        cmd = build_command(_make_config(), "/tmp/out")
        assert cmd[0] == BINARY

    @pytest.mark.parametrize("flag,expected,kwargs", [
        ("--vocabulary-file", "Vocabulary/ORBvoc.txt", {}),
        ("--settings-file", "settings.yaml", {}),
        ("--dataset", "tum", {"dataset": "tum"}),
        ("--sensor", "rgbd", {"sensor": "rgbd"}),
        ("--sensor", "mono", {"sensor": "mono"}),
        ("--sensor", "stereo", {"sensor": "stereo"}),
    ])
    def test_flag_has_expected_value(self, flag, expected, kwargs) -> None:
        cmd = build_command(_make_config(**kwargs), "/tmp/out")
        assert cmd[cmd.index(flag) + 1] == expected

    def test_no_viewer_present(self) -> None:
        assert "--no-viewer" in build_command(_make_config(), "/tmp/out")

    def test_inertial_flag_added_when_true(self) -> None:
        assert "--inertial" in build_command(_make_config(inertial=True), "/tmp/out")

    def test_inertial_flag_absent_when_false(self) -> None:
        assert "--inertial" not in build_command(_make_config(inertial=False), "/tmp/out")

    def test_command_args_included(self) -> None:
        cmd = build_command(_make_config(command_args=["--data", "/datasets/euroc-mav/V1_01_easy"]), "/tmp/out")
        assert "--data" in cmd and "/datasets/euroc-mav/V1_01_easy" in cmd

    def test_output_dir_at_end(self) -> None:
        cmd = build_command(_make_config(), "/tmp/out")
        assert cmd[-2:] == ["--output-dir", "/tmp/out"]


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# TEST_CONFIGS: validity checks                                                                     #


_VALID_DATASETS = {"euroc", "tum", "tumvi"}
_VALID_SENSORS = {"mono", "stereo", "rgbd"}


class TestConfigValidity:
    """Verify all TEST_CONFIGS have valid and consistent values."""

    @pytest.mark.parametrize("name,config", list(TEST_CONFIGS.items()))
    def test_config_is_valid(self, name: str, config: TestConfig) -> None:
        assert config.dataset in _VALID_DATASETS, f"{name}: invalid dataset"
        assert config.sensor in _VALID_SENSORS, f"{name}: invalid sensor"
        assert config.output_format in {"tum", "euroc"}, f"{name}: invalid output_format"
        assert config.name == name, f"Key '{name}' != config.name"
        assert "--data" in config.command_args, f"{name}: missing '--data'"
        stale = {"--sequence-dir", "--sequences", "--association-file"}
        assert not (stale & set(config.command_args)), f"{name}: stale flags"

    def test_inertial_configs_have_inertial_true(self) -> None:
        for name, config in TEST_CONFIGS.items():
            if "inertial" in name:
                assert config.inertial is True, f"{name} should have inertial=True"

    def test_non_inertial_configs_have_inertial_false(self) -> None:
        for name, config in TEST_CONFIGS.items():
            if "inertial" not in name:
                assert config.inertial is False, f"{name} should have inertial=False"


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# read_euroc_output: trajectory parsing                                                             #


class TestReadEurocOutput:
    """Tests for the read_euroc_output() function."""

    def test_timestamps_converted_from_ns_to_seconds(self, tmp_path) -> None:
        traj = _write_euroc(tmp_path, np.array([
            [1e9, 0, 0, 0, 0, 0, 0, 1],
            [2e9, 1, 0, 0, 0, 0, 0, 1],
            [3e9, 2, 0, 0, 0, 0, 0, 1],
        ]))
        np.testing.assert_allclose(traj.timestamps, [1.0, 2.0, 3.0])

    def test_quaternion_reordered_from_xyzw_to_wxyz(self, tmp_path) -> None:
        traj = _write_euroc(tmp_path, np.array([
            [1e9, 0, 0, 0, 0.1, 0.2, 0.3, 0.4],
            [2e9, 1, 0, 0, 0.5, 0.6, 0.7, 0.8],
        ]))
        np.testing.assert_allclose(traj.orientations_quat_wxyz[0], [0.4, 0.1, 0.2, 0.3])

    def test_positions_extracted_correctly(self, tmp_path) -> None:
        traj = _write_euroc(tmp_path, np.array([
            [1e9, 1.5, 2.5, 3.5, 0, 0, 0, 1],
            [2e9, 4.5, 5.5, 6.5, 0, 0, 0, 1],
        ]))
        np.testing.assert_allclose(traj.positions_xyz[0], [1.5, 2.5, 3.5])
        np.testing.assert_allclose(traj.positions_xyz[1], [4.5, 5.5, 6.5])

    def test_wrong_column_count_raises_value_error(self, tmp_path) -> None:
        filepath = tmp_path / "traj.txt"
        np.savetxt(str(filepath), np.array([[1e9, 0, 0, 0, 0, 0]]))
        with pytest.raises(ValueError, match="Expected 8 columns"):
            read_euroc_output(filepath)


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# compute_ate_rmse: metric computation                                                              #

_NON_COLLINEAR_POS = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1]]
_TIMESTAMPS_5 = [0.0, 1.0, 2.0, 3.0, 4.0]


class TestComputeAteRmse:
    """Tests for the compute_ate_rmse() function."""

    def test_identical_trajectories_give_zero_rmse(self) -> None:
        traj = _make_trajectory(_NON_COLLINEAR_POS[:4], [0.0, 1.0, 2.0, 3.0])
        assert compute_ate_rmse(traj, copy.deepcopy(traj), "se3") < 1e-10

    def test_constant_offset_removed_by_se3_alignment(self) -> None:
        ref = _make_trajectory(_NON_COLLINEAR_POS, _TIMESTAMPS_5)
        est_pos = np.asarray(_NON_COLLINEAR_POS, dtype=float) + [5.0, 3.0, 1.0]
        est = _make_trajectory(est_pos, _TIMESTAMPS_5)
        assert compute_ate_rmse(ref, est, "se3") < 1e-6

    def test_nonrigid_difference_gives_nonzero_rmse(self) -> None:
        ref = _make_trajectory(_NON_COLLINEAR_POS, _TIMESTAMPS_5)
        est = _make_trajectory(
            [[0, 0, 0], [1, 0.5, 0], [1, 1, 0.5], [0, 1, 0], [0, 0.5, 1]],
            _TIMESTAMPS_5,
        )
        assert compute_ate_rmse(ref, est, "se3") > 0.1

    def test_sim3_alignment_removes_scale_difference(self) -> None:
        ref = _make_trajectory(_NON_COLLINEAR_POS, _TIMESTAMPS_5)
        est = _make_trajectory(np.asarray(_NON_COLLINEAR_POS, dtype=float) * 2.0, _TIMESTAMPS_5)
        assert compute_ate_rmse(ref, est, "sim3") < 1e-6

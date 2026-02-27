#!/usr/bin/env python3
"""ORB-SLAM3 evaluation verification script.

Runs ORB-SLAM3 binaries on reference datasets and evaluates trajectory accuracy
using ATE RMSE metrics from the evo library. Supports two modes:

- baseline: Run N times, save median ATE RMSE to JSON for future comparison.
- verify:   Run N times, compare against saved baseline (threshold = factor * median).

Exit codes:
    0 - All tests pass (or baseline saved successfully).
    1 - Any test fails or an error occurs.
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Final

import numpy as np
from evo.core import metrics, sync
from evo.core.metrics import PoseRelation, StatisticsType
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

logger = logging.getLogger(__name__)

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Constants                                                                                        #

WORKING_DIR: Final[str] = "/orb_slam3"
DATASETS_DIR: Final[str] = "/datasets"
VOCABULARY_FILE: Final[str] = "Vocabulary/ORBvoc.txt"
TRAJECTORY_FILENAME: Final[str] = "CameraTrajectory.txt"
RUN_TIMEOUT_SECONDS: Final[int] = 600
DEFAULT_BASELINE_RUNS: Final[int] = 5
DEFAULT_VERIFY_RUNS: Final[int] = 3
DEFAULT_THRESHOLD_FACTOR: Final[float] = 1.5
DEFAULT_BASELINE_FILE: Final[str] = "evaluation/baseline.json"
DEFAULT_OUTPUT_FILE: Final[str] = "results/output.json"
JSON_VERSION: Final[int] = 1

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Data model                                                                                       #


@dataclass(frozen=True)
class TestConfig:
    """Configuration for a single ORB-SLAM3 evaluation test."""

    name: str
    binary: str
    output_format: str  # "tum" or "euroc"
    gt_path: str  # Relative to DATASETS_DIR
    gt_format: str  # "tum" or "euroc"
    alignment: str  # "se3" or "sim3"
    settings_file: str
    command_args: list[str] = field(default_factory=list)
    trajectory_filename: str = TRAJECTORY_FILENAME


# TUM-VI dataset base path components
_TUMVI_BASE: Final[str] = "tum-vi/dataset-corridor1_512_16/mav0"
_TUMVI_CAM0_DATA: Final[str] = f"/datasets/{_TUMVI_BASE}/cam0/data"
_TUMVI_CAM0_CSV: Final[str] = f"/datasets/{_TUMVI_BASE}/cam0/data.csv"
_TUMVI_IMU_CSV: Final[str] = f"/datasets/{_TUMVI_BASE}/imu0/data.csv"
_TUMVI_GT_PATH: Final[str] = f"{_TUMVI_BASE}/mocap0/data.csv"

# EuRoC dataset base paths
_EUROC_V1_01: Final[str] = "/datasets/euroc-mav/V1_01_easy"
_EUROC_V1_02: Final[str] = "/datasets/euroc-mav/V1_02_medium"
_EUROC_V1_01_GT: Final[str] = "euroc-mav/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv"
_EUROC_V1_02_GT: Final[str] = "euroc-mav/V1_02_medium/mav0/state_groundtruth_estimate0/data.csv"

TEST_CONFIGS: Final[dict[str, TestConfig]] = {
    # ── RGB-D TUM (2 tests, different sequences) ─────────────────────────────
    "rgbd_tum_fr1_room": TestConfig(
        name="rgbd_tum_fr1_room",
        binary="rgbd_tum",
        output_format="tum",
        gt_path="tum-rgbd-slam/rgbd_dataset_freiburg1_room/groundtruth.txt",
        gt_format="tum",
        alignment="se3",
        settings_file="Examples/RGB-D/TUM1.yaml",
        command_args=[
            "--sequence-dir",
            "/datasets/tum-rgbd-slam/rgbd_dataset_freiburg1_room",
            "--association-file",
            "Examples/RGB-D/associations/fr1_room.txt",
        ],
    ),
    "rgbd_tum_fr2_pioneer_slam": TestConfig(
        name="rgbd_tum_fr2_pioneer_slam",
        binary="rgbd_tum",
        output_format="tum",
        gt_path="tum-rgbd-slam/rgbd_dataset_freiburg2_pioneer_slam/groundtruth.txt",
        gt_format="tum",
        alignment="se3",
        settings_file="Examples/RGB-D/TUM2.yaml",
        command_args=[
            "--sequence-dir",
            "/datasets/tum-rgbd-slam/rgbd_dataset_freiburg2_pioneer_slam",
            "--association-file",
            "Examples/RGB-D/associations/fr2_pioneer_slam.txt",
        ],
    ),
    # ── Monocular (2 tests, TUM + EuRoC) ─────────────────────────────────────
    "mono_tum_fr1_room": TestConfig(
        name="mono_tum_fr1_room",
        binary="mono_tum",
        output_format="tum",
        gt_path="tum-rgbd-slam/rgbd_dataset_freiburg1_room/groundtruth.txt",
        gt_format="tum",
        alignment="sim3",
        settings_file="Examples/Monocular/TUM1.yaml",
        trajectory_filename="KeyFrameTrajectory.txt",
        command_args=[
            "--sequence-dir",
            "/datasets/tum-rgbd-slam/rgbd_dataset_freiburg1_room",
        ],
    ),
    "mono_euroc_v1_01": TestConfig(
        name="mono_euroc_v1_01",
        binary="mono_euroc",
        output_format="euroc",
        gt_path=_EUROC_V1_01_GT,
        gt_format="euroc",
        alignment="sim3",
        settings_file="Examples/Monocular/EuRoC.yaml",
        command_args=[
            "--sequences",
            _EUROC_V1_01,
            f"{_EUROC_V1_01}/mav0/cam0/data.csv",
        ],
    ),
    # ── Monocular-Inertial (2 tests, TUM-VI + EuRoC) ─────────────────────────
    "mono_inertial_tum_vi_corridor1": TestConfig(
        name="mono_inertial_tum_vi_corridor1",
        binary="mono_inertial_tum_vi",
        output_format="euroc",
        gt_path=_TUMVI_GT_PATH,
        gt_format="euroc",
        alignment="se3",
        settings_file="Examples/Monocular-Inertial/TUM-VI.yaml",
        command_args=[
            "--sequences",
            _TUMVI_CAM0_DATA,
            _TUMVI_CAM0_CSV,
            _TUMVI_IMU_CSV,
        ],
    ),
    "mono_inertial_euroc_v1_01": TestConfig(
        name="mono_inertial_euroc_v1_01",
        binary="mono_inertial_euroc",
        output_format="euroc",
        gt_path=_EUROC_V1_01_GT,
        gt_format="euroc",
        alignment="se3",
        settings_file="Examples/Monocular-Inertial/EuRoC.yaml",
        command_args=[
            "--sequences",
            _EUROC_V1_01,
            f"{_EUROC_V1_01}/mav0/cam0/data.csv",
        ],
    ),
    # ── Stereo EuRoC (2 tests, different sequences) ──────────────────────────
    "stereo_euroc_v1_01": TestConfig(
        name="stereo_euroc_v1_01",
        binary="stereo_euroc",
        output_format="euroc",
        gt_path=_EUROC_V1_01_GT,
        gt_format="euroc",
        alignment="se3",
        settings_file="Examples/Stereo/EuRoC.yaml",
        command_args=[
            "--sequences",
            _EUROC_V1_01,
            f"{_EUROC_V1_01}/mav0/cam0/data.csv",
        ],
    ),
    "stereo_euroc_v1_02": TestConfig(
        name="stereo_euroc_v1_02",
        binary="stereo_euroc",
        output_format="euroc",
        gt_path=_EUROC_V1_02_GT,
        gt_format="euroc",
        alignment="se3",
        settings_file="Examples/Stereo/EuRoC.yaml",
        command_args=[
            "--sequences",
            _EUROC_V1_02,
            f"{_EUROC_V1_02}/mav0/cam0/data.csv",
        ],
    ),
    # ── Stereo-Inertial EuRoC (2 tests, different sequences) ─────────────────
    "stereo_inertial_euroc_v1_01": TestConfig(
        name="stereo_inertial_euroc_v1_01",
        binary="stereo_inertial_euroc",
        output_format="euroc",
        gt_path=_EUROC_V1_01_GT,
        gt_format="euroc",
        alignment="se3",
        settings_file="Examples/Stereo-Inertial/EuRoC.yaml",
        command_args=[
            "--sequences",
            _EUROC_V1_01,
            f"{_EUROC_V1_01}/mav0/cam0/data.csv",
        ],
    ),
    "stereo_inertial_euroc_v1_02": TestConfig(
        name="stereo_inertial_euroc_v1_02",
        binary="stereo_inertial_euroc",
        output_format="euroc",
        gt_path=_EUROC_V1_02_GT,
        gt_format="euroc",
        alignment="se3",
        settings_file="Examples/Stereo-Inertial/EuRoC.yaml",
        command_args=[
            "--sequences",
            _EUROC_V1_02,
            f"{_EUROC_V1_02}/mav0/cam0/data.csv",
        ],
    ),
}

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Trajectory I/O                                                                                   #


def read_euroc_output(filepath: Path) -> PoseTrajectory3D:
    """Read SaveTrajectoryEuRoC output and convert to evo trajectory.

    The EuRoC output format is space-delimited:
        timestamp_ns tx ty tz qx qy qz qw

    This function converts timestamps from nanoseconds to seconds and reorders
    the quaternion from (qx, qy, qz, qw) to (qw, qx, qy, qz) as expected
    by evo's PoseTrajectory3D.

    Args:
        filepath: Path to the trajectory file.

    Returns:
        PoseTrajectory3D with correct timestamps and quaternion ordering.

    Raises:
        FileNotFoundError: If the trajectory file does not exist.
        ValueError: If the file has unexpected format.
    """
    raw = np.loadtxt(str(filepath))
    if raw.ndim != 2 or raw.shape[1] != 8:
        msg = (
            f"Expected 8 columns (ts tx ty tz qx qy qz qw), "
            f"got shape {raw.shape} in {filepath}"
        )
        raise ValueError(msg)

    timestamps = raw[:, 0] / 1e9  # ns -> seconds
    positions = raw[:, 1:4]  # tx, ty, tz
    # Reorder quaternion: (qx, qy, qz, qw) -> (qw, qx, qy, qz)
    quat_xyzw = raw[:, 4:8]
    quat_wxyz = np.column_stack(
        [quat_xyzw[:, 3], quat_xyzw[:, 0], quat_xyzw[:, 1], quat_xyzw[:, 2]]
    )

    return PoseTrajectory3D(
        positions_xyz=positions,
        orientations_quat_wxyz=quat_wxyz,
        timestamps=timestamps,
    )


def read_estimated_trajectory(filepath: Path, output_format: str) -> PoseTrajectory3D:
    """Read an estimated trajectory file based on its output format.

    Args:
        filepath: Path to the trajectory file.
        output_format: Either "tum" or "euroc".

    Returns:
        PoseTrajectory3D for the estimated trajectory.
    """
    if output_format == "tum":
        return file_interface.read_tum_trajectory_file(str(filepath))
    if output_format == "euroc":
        return read_euroc_output(filepath)
    msg = f"Unknown output format: {output_format}"
    raise ValueError(msg)


def read_ground_truth(gt_path: Path, gt_format: str) -> PoseTrajectory3D:
    """Read a ground truth trajectory file based on its format.

    Args:
        gt_path: Absolute path to the ground truth file.
        gt_format: Either "tum" or "euroc".

    Returns:
        PoseTrajectory3D for the ground truth trajectory.
    """
    if gt_format == "tum":
        return file_interface.read_tum_trajectory_file(str(gt_path))
    if gt_format == "euroc":
        return file_interface.read_euroc_csv_trajectory(str(gt_path))
    msg = f"Unknown ground truth format: {gt_format}"
    raise ValueError(msg)


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# ATE evaluation                                                                                   #


def compute_ate_rmse(
    traj_ref: PoseTrajectory3D,
    traj_est: PoseTrajectory3D,
    alignment: str,
) -> float:
    """Compute Absolute Trajectory Error (ATE) RMSE.

    Synchronizes trajectories by timestamp, aligns the estimated trajectory
    to the reference, and computes the RMSE of the translational APE.

    Args:
        traj_ref: Reference (ground truth) trajectory.
        traj_est: Estimated trajectory.
        alignment: Alignment type - "se3" (rigid) or "sim3" (with scale).

    Returns:
        ATE RMSE value in meters.
    """
    traj_ref_synced, traj_est_synced = sync.associate_trajectories(traj_ref, traj_est)

    traj_est_aligned = copy.deepcopy(traj_est_synced)
    if alignment == "sim3":
        traj_est_aligned.align(traj_ref_synced, correct_scale=True)
    else:
        traj_est_aligned.align(traj_ref_synced, correct_scale=False)

    data = (traj_ref_synced, traj_est_aligned)
    ape_metric = metrics.APE(PoseRelation.translation_part)
    ape_metric.process_data(data)
    return float(ape_metric.get_statistic(StatisticsType.rmse))


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Binary execution                                                                                 #


def build_command(config: TestConfig, output_dir: str) -> list[str]:
    """Build the subprocess command for an ORB-SLAM3 binary.

    Args:
        config: Test configuration.
        output_dir: Temporary directory for output files.

    Returns:
        Command as a list of strings.
    """
    cmd = [
        config.binary,
        "--vocabulary-file",
        VOCABULARY_FILE,
        "--no-viewer",
        "--settings-file",
        config.settings_file,
    ]
    cmd.extend(config.command_args)
    cmd.extend(["--output-dir", output_dir])
    return cmd


def run_single(config: TestConfig) -> float:
    """Execute one run of a test and return the ATE RMSE.

    Creates a temporary directory for output, runs the binary, reads the
    resulting trajectory, and computes ATE RMSE against ground truth.

    Args:
        config: Test configuration.

    Returns:
        ATE RMSE value in meters.

    Raises:
        subprocess.TimeoutExpired: If the binary exceeds the timeout.
        subprocess.CalledProcessError: If the binary returns non-zero.
        FileNotFoundError: If the output trajectory file is missing.
    """
    output_dir = tempfile.mkdtemp(prefix=f"orbslam3_{config.name}_")
    try:
        cmd = build_command(config, output_dir)
        logger.info("Running: %s", " ".join(cmd))

        subprocess.run(
            cmd,
            cwd=WORKING_DIR,
            timeout=RUN_TIMEOUT_SECONDS,
            check=True,
        )

        traj_file = Path(output_dir) / config.trajectory_filename
        if not traj_file.exists():
            msg = f"Trajectory file not found: {traj_file}"
            raise FileNotFoundError(msg)

        traj_est = read_estimated_trajectory(traj_file, config.output_format)
        gt_path = Path(DATASETS_DIR) / config.gt_path
        traj_ref = read_ground_truth(gt_path, config.gt_format)

        rmse = compute_ate_rmse(traj_ref, traj_est, config.alignment)
        logger.info("  ATE RMSE: %.6f m", rmse)
        return rmse
    finally:
        shutil.rmtree(output_dir, ignore_errors=True)


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Git helpers                                                                                      #


def get_git_commit() -> str:
    """Get the short git commit hash of the current HEAD.

    Returns:
        Short commit hash string, or "unknown" if git is not available.
    """
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
            cwd=WORKING_DIR,
        )
        return result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Test runner                                                                                      #


def run_test(config: TestConfig, num_runs: int) -> list[float]:
    """Run a test configuration multiple times and collect RMSE values.

    Args:
        config: Test configuration.
        num_runs: Number of times to run the test.

    Returns:
        List of ATE RMSE values, one per successful run.
        Failed runs are represented as NaN.
    """
    results: list[float] = []
    for i in range(num_runs):
        logger.info("Test '%s' run %d/%d", config.name, i + 1, num_runs)
        try:
            rmse = run_single(config)
            results.append(rmse)
        except subprocess.TimeoutExpired:
            logger.error(
                "Test '%s' run %d timed out after %ds",
                config.name,
                i + 1,
                RUN_TIMEOUT_SECONDS,
            )
            results.append(float("nan"))
        except (subprocess.CalledProcessError, FileNotFoundError, ValueError) as exc:
            logger.error("Test '%s' run %d failed: %s", config.name, i + 1, exc)
            results.append(float("nan"))
    return results


def select_tests(test_names: list[str] | None) -> list[TestConfig]:
    """Select test configurations by name.

    Args:
        test_names: List of test names to run, or None to run all.

    Returns:
        List of TestConfig objects.

    Raises:
        SystemExit: If any requested test name is unknown.
    """
    if test_names is None:
        return list(TEST_CONFIGS.values())

    configs: list[TestConfig] = []
    for name in test_names:
        if name not in TEST_CONFIGS:
            logger.error(
                "Unknown test '%s'. Available: %s",
                name,
                ", ".join(TEST_CONFIGS.keys()),
            )
            sys.exit(1)
        configs.append(TEST_CONFIGS[name])
    return configs


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Modes                                                                                            #


def mode_baseline(
    runs: int,
    test_names: list[str] | None,
    baseline_file: Path,
    output_file: Path,
    threshold_factor: float,
) -> int:
    """Run baseline mode: execute tests and save median results.

    Args:
        runs: Number of runs per test.
        test_names: Optional list of test names to run.
        baseline_file: Path to save/update baseline JSON.
        output_file: Path to save detailed output JSON.
        threshold_factor: Multiplier for median to set threshold.

    Returns:
        Exit code (0 for success, 1 for any failure).
    """
    configs = select_tests(test_names)
    git_commit = get_git_commit()
    all_pass = True
    test_results: dict[str, dict] = {}

    for config in configs:
        logger.info("=== Baseline: %s (%d runs) ===", config.name, runs)
        rmse_values = run_test(config, runs)

        valid_values = [v for v in rmse_values if not np.isnan(v)]
        if not valid_values:
            logger.error("Test '%s': all runs failed", config.name)
            test_results[config.name] = {
                "runs": rmse_values,
                "median_rmse": None,
                "threshold": None,
                "status": "fail",
                "alignment": config.alignment,
            }
            all_pass = False
            continue

        median_rmse = median(valid_values)
        threshold = median_rmse * threshold_factor

        logger.info(
            "Test '%s': median=%.6f, threshold=%.6f",
            config.name,
            median_rmse,
            threshold,
        )

        test_results[config.name] = {
            "runs": rmse_values,
            "median_rmse": median_rmse,
            "threshold": threshold,
            "status": "pass",
            "alignment": config.alignment,
        }

    output = {
        "version": JSON_VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_commit,
        "mode": "baseline",
        "threshold_factor": threshold_factor,
        "tests": test_results,
    }

    _write_json(output_file, output)
    _write_json(baseline_file, output)
    logger.info("Baseline saved to %s", baseline_file)

    return 0 if all_pass else 1


def mode_verify(
    runs: int,
    test_names: list[str] | None,
    baseline_file: Path,
    output_file: Path,
    threshold_factor: float,
) -> int:
    """Run verify mode: execute tests and compare against baseline.

    Args:
        runs: Number of runs per test.
        test_names: Optional list of test names to run.
        baseline_file: Path to read baseline JSON from.
        output_file: Path to save detailed output JSON.
        threshold_factor: Multiplier for baseline median to set threshold.

    Returns:
        Exit code (0 for all pass, 1 for any failure).
    """
    if not baseline_file.exists():
        logger.error("Baseline file not found: %s", baseline_file)
        return 1

    baseline = _read_json(baseline_file)
    if baseline is None:
        logger.error("Failed to read baseline file: %s", baseline_file)
        return 1

    baseline_tests = baseline.get("tests", {})
    configs = select_tests(test_names)
    git_commit = get_git_commit()
    all_pass = True
    test_results: dict[str, dict] = {}

    for config in configs:
        if config.name not in baseline_tests:
            logger.error(
                "Test '%s' not found in baseline. Run baseline first.",
                config.name,
            )
            test_results[config.name] = {
                "runs": [],
                "median_rmse": None,
                "threshold": None,
                "status": "fail",
                "alignment": config.alignment,
            }
            all_pass = False
            continue

        baseline_entry = baseline_tests[config.name]
        baseline_median = baseline_entry.get("median_rmse")
        if baseline_median is None:
            logger.error(
                "Test '%s' has no valid baseline median. Re-run baseline.",
                config.name,
            )
            test_results[config.name] = {
                "runs": [],
                "median_rmse": None,
                "threshold": None,
                "status": "fail",
                "alignment": config.alignment,
            }
            all_pass = False
            continue

        threshold = baseline_median * threshold_factor

        logger.info(
            "=== Verify: %s (%d runs, threshold=%.6f) ===",
            config.name,
            runs,
            threshold,
        )

        rmse_values = run_test(config, runs)
        valid_values = [v for v in rmse_values if not np.isnan(v)]

        if not valid_values:
            logger.error("Test '%s': all runs failed", config.name)
            test_results[config.name] = {
                "runs": rmse_values,
                "median_rmse": None,
                "threshold": threshold,
                "baseline_median": baseline_median,
                "status": "fail",
                "alignment": config.alignment,
            }
            all_pass = False
            continue

        current_median = median(valid_values)
        passed = current_median <= threshold
        status = "pass" if passed else "fail"

        if passed:
            logger.info(
                "Test '%s': PASS (median=%.6f <= threshold=%.6f)",
                config.name,
                current_median,
                threshold,
            )
        else:
            logger.error(
                "Test '%s': FAIL (median=%.6f > threshold=%.6f)",
                config.name,
                current_median,
                threshold,
            )
            all_pass = False

        test_results[config.name] = {
            "runs": rmse_values,
            "median_rmse": current_median,
            "threshold": threshold,
            "baseline_median": baseline_median,
            "status": status,
            "alignment": config.alignment,
        }

    output = {
        "version": JSON_VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_commit,
        "mode": "verify",
        "threshold_factor": threshold_factor,
        "tests": test_results,
    }

    _write_json(output_file, output)
    logger.info("Results saved to %s", output_file)

    return 0 if all_pass else 1


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# JSON I/O                                                                                         #


def _write_json(filepath: Path, data: dict) -> None:
    """Write a dictionary to a JSON file, creating parent directories.

    Args:
        filepath: Path to the output JSON file.
        data: Dictionary to serialize.
    """
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with filepath.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, default=_json_serializer)
    logger.info("Wrote %s", filepath)


def _json_serializer(obj: object) -> object:
    """Custom JSON serializer for numpy and special float types.

    Args:
        obj: Object to serialize.

    Returns:
        JSON-serializable representation.

    Raises:
        TypeError: If the object is not serializable.
    """
    if isinstance(obj, float) and np.isnan(obj):
        return None
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    msg = f"Object of type {type(obj)} is not JSON serializable"
    raise TypeError(msg)


def _read_json(filepath: Path) -> dict | None:
    """Read a JSON file and return as a dictionary.

    Args:
        filepath: Path to the JSON file.

    Returns:
        Parsed dictionary, or None if reading fails.
    """
    try:
        with filepath.open("r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError) as exc:
        logger.error("Error reading %s: %s", filepath, exc)
        return None


# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# CLI                                                                                              #


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser for the verification script.

    Returns:
        Configured ArgumentParser.
    """
    parser = argparse.ArgumentParser(
        description="ORB-SLAM3 evaluation verification tool.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    # -- baseline subcommand --
    baseline_parser = subparsers.add_parser(
        "baseline",
        help="Run tests and save median ATE RMSE as baseline.",
    )
    baseline_parser.add_argument(
        "--runs",
        type=int,
        default=DEFAULT_BASELINE_RUNS,
        help=f"Number of runs per test (default: {DEFAULT_BASELINE_RUNS}).",
    )
    baseline_parser.add_argument(
        "--tests",
        nargs="+",
        metavar="NAME",
        default=None,
        help="Test names to run (default: all).",
    )
    baseline_parser.add_argument(
        "--baseline-file",
        type=Path,
        default=Path(DEFAULT_BASELINE_FILE),
        help=f"Path to save baseline JSON (default: {DEFAULT_BASELINE_FILE}).",
    )
    baseline_parser.add_argument(
        "--output-file",
        type=Path,
        default=Path(DEFAULT_OUTPUT_FILE),
        help=f"Path to save output JSON (default: {DEFAULT_OUTPUT_FILE}).",
    )
    baseline_parser.add_argument(
        "--threshold-factor",
        type=float,
        default=DEFAULT_THRESHOLD_FACTOR,
        help=f"Threshold multiplier for median (default: {DEFAULT_THRESHOLD_FACTOR}).",
    )

    # -- verify subcommand --
    verify_parser = subparsers.add_parser(
        "verify",
        help="Run tests and verify against saved baseline.",
    )
    verify_parser.add_argument(
        "--runs",
        type=int,
        default=DEFAULT_VERIFY_RUNS,
        help=f"Number of runs per test (default: {DEFAULT_VERIFY_RUNS}).",
    )
    verify_parser.add_argument(
        "--tests",
        nargs="+",
        metavar="NAME",
        default=None,
        help="Test names to run (default: all).",
    )
    verify_parser.add_argument(
        "--baseline-file",
        type=Path,
        default=Path(DEFAULT_BASELINE_FILE),
        help=f"Path to read baseline JSON (default: {DEFAULT_BASELINE_FILE}).",
    )
    verify_parser.add_argument(
        "--output-file",
        type=Path,
        default=Path(DEFAULT_OUTPUT_FILE),
        help=f"Path to save output JSON (default: {DEFAULT_OUTPUT_FILE}).",
    )
    verify_parser.add_argument(
        "--threshold-factor",
        type=float,
        default=DEFAULT_THRESHOLD_FACTOR,
        help=f"Threshold multiplier for baseline median (default: {DEFAULT_THRESHOLD_FACTOR}).",
    )

    return parser


def main() -> int:
    """Entry point for the verification script.

    Returns:
        Exit code (0 for success, 1 for failure).
    """
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        stream=sys.stderr,
    )

    parser = build_parser()
    args = parser.parse_args()

    if args.mode == "baseline":
        return mode_baseline(
            runs=args.runs,
            test_names=args.tests,
            baseline_file=args.baseline_file,
            output_file=args.output_file,
            threshold_factor=args.threshold_factor,
        )
    if args.mode == "verify":
        return mode_verify(
            runs=args.runs,
            test_names=args.tests,
            baseline_file=args.baseline_file,
            output_file=args.output_file,
            threshold_factor=args.threshold_factor,
        )

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())

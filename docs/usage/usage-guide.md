# Usage Guide

## Unified Offline Binary

`orb_slam3_offline` replaces the 12 legacy per-dataset/per-sensor binaries with a single CLI tool.

### CLI Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--dataset` | Yes | | Dataset type: `euroc`, `tum`, or `tumvi` |
| `--sensor` | Yes | | Sensor mode: `mono`, `stereo`, or `rgbd` |
| `--vocabulary-file` | Yes | | Path to `ORBvoc.txt` vocabulary file |
| `--settings-file` | Yes | | Path to YAML camera/ORB settings file |
| `--data` | Yes | | Path(s) to dataset directories (multi-token) |
| `--inertial` | No | off | Enable IMU integration |
| `--no-viewer` | No | off | Disable the Pangolin viewer |
| `--output-dir` | No | `/tmp` | Output directory for trajectory files |
| `-h`, `--help` | No | | Show help message |

### Dataset / Sensor Support Matrix

| Dataset | Mono | Stereo | RGB-D | Mono+IMU | Stereo+IMU |
|---------|------|--------|-------|----------|------------|
| EuRoC   | Y    | Y      | --    | Y        | Y          |
| TUM     | Y    | --     | Y     | --       | --         |
| TUM-VI  | --   | --     | --    | Y        | Y          |

Invalid combinations (e.g. `--dataset tum --sensor stereo`) are rejected at startup.

### Example Commands

**EuRoC Stereo-Inertial:**

```sh
orb_slam3_offline \
  --dataset euroc --sensor stereo --inertial \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/Stereo-Inertial/EuRoC.yaml \
  --data /datasets/euroc-mav/V1_01_easy \
  --output-dir /tmp/results
```

**TUM RGB-D:**

```sh
orb_slam3_offline \
  --dataset tum --sensor rgbd \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/RGB-D/TUM1.yaml \
  --data /datasets/tum-rgbd-slam/rgbd_dataset_freiburg1_room \
  --output-dir /tmp/results
```

**TUM-VI Monocular-Inertial:**

```sh
orb_slam3_offline \
  --dataset tumvi --sensor mono --inertial \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/Monocular-Inertial/TUM-VI.yaml \
  --data /datasets/tum-vi/dataset-corridor1_512_16 \
  --output-dir /tmp/results
```

**EuRoC Monocular (headless):**

```sh
orb_slam3_offline \
  --dataset euroc --sensor mono --no-viewer \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/Monocular/EuRoC.yaml \
  --data /datasets/euroc-mav/V1_01_easy \
  --output-dir /tmp/results
```

### Output Files

Saved to the `--output-dir` path:

| File | Content | When Written |
|------|---------|--------------|
| `CameraTrajectory.txt` | Full camera trajectory (all frames) | All modes except TUM Mono |
| `KeyFrameTrajectory.txt` | Keyframe-only trajectory | All modes |

**Format by dataset:**

- **EuRoC / TUM-VI** -- EuRoC format with nanosecond timestamps, IMU body-frame poses, biggest-map selection.
- **TUM** -- TUM format: `timestamp tx ty tz qx qy qz qw` (seconds). Monocular TUM saves only `KeyFrameTrajectory.txt`.

Multi-sequence runs also save per-sequence SubMap trajectories to `<output-dir>/submap/`.

### YAML Settings Files

Located in `Examples/<SensorMode>/`:

- Camera intrinsics (`fx`, `fy`, `cx`, `cy`, distortion coefficients)
- ORB parameters (`nFeatures`, `scaleFactor`, `nLevels`, `iniThFAST`, `minThFAST`)
- IMU calibration (noise density, random walk, `Tbc` transform) -- inertial modes only
- Viewer settings (window size, viewpoint)

### Logging

- Console log level: set via `SPDLOG_LEVEL` env var (e.g. `SPDLOG_LEVEL=debug`)
- Per-logger levels supported via spdlog argv syntax (`--spdlog_level=ORB-SLAM3=debug`)
- File sink: `/tmp/orb_slam3_offline.log`

---

## Evaluation Pipeline

### verify.py

Python evaluation script using the [evo](https://github.com/MichaelGrupp/evo) trajectory analysis toolkit. Runs inside a Docker container built from the `evo` stage of the Dockerfile.

**Modes:**

| Mode | Description |
|------|-------------|
| `baseline` | Run each test N times, save median ATE RMSE to JSON |
| `verify` | Run each test N times, compare against saved baseline (pass if `result < factor * median`) |

**Usage (inside evo container):**

```sh
# Create baseline (5 runs per test)
python3 evaluation/verify.py baseline --runs 5

# Verify against baseline (3 runs per test)
python3 evaluation/verify.py verify --runs 3

# Run specific tests only
python3 evaluation/verify.py verify --runs 3 \
  --tests rgbd_tum_fr1_room stereo_euroc_v1_01
```

### Available Tests

| Test Name | Dataset | Sensor | Alignment |
|-----------|---------|--------|-----------|
| `rgbd_tum_fr1_room` | TUM | RGB-D | SE3 |
| `rgbd_tum_fr2_pioneer_slam` | TUM | RGB-D | SE3 |
| `mono_tum_fr1_room` | TUM | Mono | Sim3 |
| `mono_euroc_v1_01` | EuRoC | Mono | Sim3 |
| `mono_inertial_tum_vi_corridor1` | TUM-VI | Mono+IMU | SE3 |
| `mono_inertial_euroc_v1_01` | EuRoC | Mono+IMU | SE3 |
| `stereo_euroc_v1_01` | EuRoC | Stereo | SE3 |
| `stereo_euroc_v1_02` | EuRoC | Stereo | SE3 |
| `stereo_inertial_euroc_v1_01` | EuRoC | Stereo+IMU | SE3 |
| `stereo_inertial_euroc_v1_02` | EuRoC | Stereo+IMU | SE3 |

Mono tests use Sim3 alignment (corrects scale); all others use SE3 (rigid body).

### Full Evaluation Pipeline Script

`scripts/eval_pipeline.sh` automates end-to-end regression testing:

1. Builds the evo Docker image at a baseline commit, runs baseline (10 runs/test)
2. Builds the evo Docker image at HEAD, runs verify (3 runs/test)
3. Reports PASS/FAIL per test against the baseline median

```sh
./scripts/eval_pipeline.sh
# Results: results/verify-3x-output.json
# Log:     results/eval-pipeline.log
```

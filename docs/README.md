# ORB-SLAM3 Documentation

Modernized C++20 fork of ORB-SLAM3 — a visual-inertial SLAM system supporting monocular, stereo, and RGB-D cameras with optional IMU fusion.

## Contents

| Document | Description |
|----------|-------------|
| [Architecture](architecture/architecture.md) | System design, threading model, core components |
| [API Reference](interfaces/api-reference.md) | Public API: System class, sensor modes, DatasetRunner |
| [System Diagrams](diagrams/system-diagrams.md) | Mermaid diagrams: component, class, sequence, state |
| [Build Guide](building/build-guide.md) | Docker build, CMake setup, dependencies, tests |
| [Usage Guide](usage/usage-guide.md) | Running `orb_slam3_offline`, datasets, evaluation |
| [System Status](SYSTEM_STATUS.md) | Project progress, known issues, roadmap |

## Quick Start

```bash
# Build runtime image
docker compose build orb-slam3

# Run stereo-inertial on EuRoC
docker compose run orb-slam3
orb_slam3_offline \
  --dataset euroc --sensor stereo --inertial \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/Stereo-Inertial/EuRoC.yaml \
  --data /datasets/euroc-mav/V1_01_easy \
  --output-dir /tmp/results
```

## Project Structure

```
include/              Public headers
  CameraModels/       Camera model headers (Pinhole, KannalaBrandt8)
  Common/             Dataset runner headers
src/                  Source files (mirrors include/)
  CameraModels/       Camera model implementations
  Common/             Dataset runner implementations
  orb_slam3_offline.cc  Unified offline binary
test/                 GoogleTest test files (6 tests)
Examples/             YAML configs and calibration data per sensor mode
Thirdparty/           In-tree dependencies (DBoW2, g2o, Sophus)
Vocabulary/           ORB vocabulary (ORBvoc.txt)
evaluation/           Evaluation pipeline (verify.py, baseline.json)
docs/                 This documentation
```

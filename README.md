# ORB-SLAM3 (C++20 Modernized Fork)

This is a modernized fork of the [original ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), refactored for C++20 compatibility, improved code readability, and easier maintenance.

## Key Improvements (Done)

- ORB_SLAM3 namespace wrapping across all source files
- Optimized include directives to reduce compilation time and improve build efficiency
- clang-format applied for consistent code formatting
- spdlog integration for structured logging (replacing most cout/cerr)
- Basic exception handling with try/catch patterns
- Common/ utility extraction (EuRoC, TUM, KITTI, RealSense, TUMVI helpers)
- C++20 standard enabled in CMake
- GTest integration with 5 initial test files

## Docker

**Prerequisites:** Docker Engine 20.10+, Docker Compose v2+, X11 server (for Pangolin GUI).

### Build

```bash
docker compose build orb-slam3       # Runtime image (executables + shared libs only)
docker compose build orb-slam3-dev   # Dev image (full build environment)
```

### Runtime

```bash
xhost +local:docker
docker compose run --rm orb-slam3

# Example — run RGB-D on TUM fr2_large_with_loop:
rgbd_tum \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/RGB-D/TUM2.yaml \
  --sequence-dir /datasets/tum-rgbd-slam/rgbd_dataset_freiburg2_large_with_loop \
  --association-file Examples/RGB-D/associations/fr2_large_with_loop.txt

xhost -local:docker
```

Custom dataset path: `DATASETS_DIR=/path/to/datasets docker compose run --rm orb-slam3`

All example binaries support `--no-viewer` to disable the Pangolin GUI (useful for headless environments or benchmarking):

```bash
rgbd_tum \
  --no-viewer \
  --vocabulary-file Vocabulary/ORBvoc.txt \
  --settings-file Examples/RGB-D/TUM2.yaml \
  --sequence-dir /datasets/tum-rgbd-slam/rgbd_dataset_freiburg2_large_with_loop \
  --association-file Examples/RGB-D/associations/fr2_large_with_loop.txt
```

### Development

Source tree is bind-mounted at `/orb_slam3` — edits on the host are immediately visible.
Thirdparty dependencies are pre-installed to `/usr/local` and unaffected by the mount.

```bash
xhost +local:docker
docker compose run --rm orb-slam3-dev

# Build:
cmake -B build -S . -GNinja -DCMAKE_BUILD_TYPE=Release
ninja -C build -j"$(nproc)"

# Test:
ctest --test-dir build --output-on-failure

xhost -local:docker
```

The build produces `build/compile_commands.json` for clangd/clang-tidy (visible on the host via the bind mount).

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | `:0` | X11 display for Pangolin GUI |
| `DATASETS_DIR` | `./datasets` | Host path mounted at `/datasets` |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Mesa software rendering (no GPU required) |


### Evaluation

The `orb-slam3-evo` service provides headless trajectory evaluation using the [evo](https://github.com/MichaelGrupp/evo) toolkit. No X11 server is required.

```bash
# Build the evo image (incremental, reuses cached runtime layer):
docker compose build orb-slam3-evo

# Run a single-run baseline on one test:
docker compose run --rm orb-slam3-evo \
  python3 -u evaluation/verify.py baseline \
    --tests rgbd_tum_fr2_large_with_loop --runs 1

# Verify against baseline:
docker compose run --rm orb-slam3-evo \
  python3 -u evaluation/verify.py verify

# Run a specific binary headlessly:
docker compose run --rm orb-slam3-evo \
  rgbd_tum --no-viewer \
    --vocabulary-file Vocabulary/ORBvoc.txt \
    --settings-file Examples/RGB-D/TUM2.yaml \
    --sequence-dir /datasets/tum-rgbd-slam/rgbd_dataset_freiburg2_large_with_loop \
    --association-file Examples/RGB-D/associations/fr2_large_with_loop.txt
```

Results are written to `./results/` on the host (mounted at `/orb_slam3/results` inside the container).

| Variable | Default | Description |
|----------|---------|-------------|
| `DATASETS_DIR` | `./datasets` | Host path mounted read-only at `/datasets` |
| `PYTHONUNBUFFERED` | `1` | Flush Python output in real time |

## Modernization Roadmap

### Phase 0: Mechanical Cleanup
- [ ] Replace `static_cast<T*>(NULL)` with `nullptr` (76 occurrences, 10 src files)
- [ ] Remove all `this->` member access (14 occurrences, 3 files)
- [ ] Convert `#ifndef` header guards to `#pragma once` (31 headers)
- [ ] Replace `usleep()` with `std::this_thread::sleep_for()` (39 calls)
- [ ] Replace remaining `cout`/`cerr` with spdlog (88 occurrences)
- [ ] Convert `#define FRAME_GRID_*` to `inline constexpr` (Frame.h)
- [ ] Replace `sprintf` with `fmt::format` (1 occurrence)

### Phase 1: CMake Modernization
- [x] Remove `Examples_old/` build targets and directory
- [x] Convert to per-target `target_include_directories()`
- [x] Replace hardcoded DBoW2/g2o `.so` paths with imported targets
- [x] Add `CMAKE_EXPORT_COMPILE_COMMANDS`
- [ ] Add ASan/UBSan and clang-tidy CMake options
- [ ] Add `.clang-tidy` configuration

### Phase 2: Smart Pointers and RAII
- [ ] Convert System-owned subsystems to `std::unique_ptr` (10 raw pointers)
- [ ] Convert `std::thread*` to `std::unique_ptr<std::thread>` (4 pointers)
- [ ] Convert Tracking-owned extractors/IMU to `std::unique_ptr` (5 pointers)
- [ ] Eliminate remaining `delete` calls (7 sites across 5 files)

### Phase 3: Optimizer.cc Decomposition
- [ ] Extract `BundleAdjustmentOptimizer.cc` (~1,600 lines)
- [ ] Extract `PoseOptimizer.cc` (~1,200 lines)
- [ ] Extract `GraphOptimizer.cc` (~1,100 lines)
- [ ] Extract `InertialOptimizer.cc` (~1,650 lines)
- [ ] Update CMakeLists.txt and remove monolithic `Optimizer.cc`

### Phase 4: Core/Integration Architecture Split
- [ ] Remove Pangolin dependency from `Map.h` (extract GLubyte* thumbnail)
- [ ] Extract `boost::serialization` from 10 core headers into adapters
- [ ] Create `core/` + `integration/` directory structure
- [ ] Define two CMake targets: `ORB_SLAM3_core` (pure C++) and `ORB_SLAM3` (integration)
- [ ] Abstract visualization with `IFrameDrawer`/`IMapDrawer` interfaces

### Phase 5: Test Expansion
- [ ] Add `Converter_test.cc` (Mat/Eigen round-trip, descriptor conversions)
- [ ] Add `GeometricTools_test.cc` (ComputeF12, Triangulate)
- [ ] Add `ImuTypes_test.cc` (preintegration, bias arithmetic)
- [ ] Add `ORBextractor_test.cc` (feature extraction, scale pyramid)
- [ ] Add `ORBmatcher_test.cc` (descriptor distance, thresholds)
- [ ] Add `TwoViewReconstruction_test.cc` (homography, fundamental matrix)
- [ ] Add `PoseOptimizer_test.cc` + `BundleAdjustmentOptimizer_test.cc`
- [ ] Add CMake coverage reporting (lcov/gcovr, target: 80% on core/)

### Phase 6: C++20 Features
- [ ] Add `CameraModel` concept with `static_assert` validation
- [ ] Replace `std::thread` with `std::jthread` + `std::stop_token`
- [ ] Use `std::expected` for optimizer return types
- [ ] Use `std::span` for read-only vector parameters
- [ ] Replace `#ifdef REGISTER_TIMES` with `if constexpr`

### Phase 7: Thread Safety
- [ ] Convert `bool mb*` flags to `std::atomic<bool>` (~25 flags)
- [ ] Convert read-heavy mutexes to `std::shared_mutex` (Map, KeyFrame, MapPoint, Atlas)
- [ ] Full ThreadSanitizer audit with zero warnings

### Phase 8: Naming Convention Alignment (Optional)
- [ ] Remove Hungarian notation (`mp*`, `mb*`, `mv*` → `snake_case_`)
- [ ] Align constants to `kScreamingCase`
- [ ] Execute in leaf-to-root order using clang-rename

## Features

<!-- TODO: add this -->

## Contribution

Contributions are welcome! Please open issues or submit pull requests for improvements or bug fixes.

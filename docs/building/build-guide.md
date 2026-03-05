# Build Guide

## Docker-Based Build (Recommended)

### Prerequisites

- Docker and Docker Compose
- X11 forwarding for visualization (`DISPLAY` environment variable set)

### Dockerfile Stages

The multi-stage `Dockerfile` builds dependencies in parallel, then merges them:

| # | Stage | Base | Purpose |
|---|-------|------|---------|
| 1 | **base** | ubuntu:24.04 | System packages: build-essential, cmake, ninja-build, OpenGL/EGL/Wayland/X11 libs, image codecs, Python3, Boost, Eigen3, OpenCV, spdlog, GTest, OpenSSL, Mesa utils |
| 2 | **build-pangolin** | base | Builds [Pangolin v0.9.4](https://github.com/stevenlovegrove/Pangolin) from source |
| 3 | **build-dbow2** | base | Builds in-tree `Thirdparty/DBoW2` |
| 4 | **build-g2o** | base | Builds in-tree `Thirdparty/g2o` |
| 5 | **build-sophus** | base | Builds in-tree `Thirdparty/Sophus` |
| 6 | **deps** | base | Merges all pre-built deps into `/usr/local`, extracts ORB vocabulary |
| 7 | **dev** | deps | Development image with deps + clangd + curl + wget |
| 8 | **builder** | deps | Compiles ORB-SLAM3 (CMake Release), installs to `/usr/local` |
| 9 | **runtime** | ubuntu:24.04 | Minimal image: shared libs, binaries, vocabulary, example configs |
| 10 | **evo** | runtime | Runtime + Python3 + [evo](https://github.com/MichaelGrupp/evo) evaluation toolkit |

### Docker Compose Services

Defined in `compose.yml`:

| Service | Target | Purpose |
|---------|--------|---------|
| `orb-slam3` | runtime | Run SLAM with pre-built binary |
| `orb-slam3-dev` | dev | Development with source mounted |
| `orb-slam3-evo` | evo | Evaluation with evo toolkit |

### Environment Variables

Set in `.env` or shell (defaults in parentheses):

| Variable | Default | Purpose |
|----------|---------|---------|
| `SRC_DIR` | `.` | Source directory for Docker context |
| `DATASETS_DIR` | `./datasets` | Datasets mount point (read-only) |
| `RESULTS_DIR` | `./results` | Results output (evo service) |
| `EVALUATION_DIR` | `./evaluation` | Evaluation scripts (evo service) |
| `DISPLAY` | `:0` | X11 display for visualization |
| `XAUTHORITY` | `$HOME/.Xauthority` | X11 auth file |

### Build and Run

```bash
# Build runtime image
docker compose build orb-slam3

# Build dev image
docker compose build orb-slam3-dev

# Enter dev container (source mounted at /orb-slam3)
docker compose run orb-slam3-dev
```

### Inside Dev Container

```bash
cmake -B build -S . -GNinja -DCMAKE_BUILD_TYPE=Release
ninja -C build -j4   # Use -j4 max to avoid OOM
ctest --test-dir build
```

---

## CMake Build (Direct)

### Dependencies

OpenCV 4.4+, Eigen3 3.1+, Pangolin, Boost (program_options, serialization), spdlog, GTest, DBoW2, g2o, Sophus, OpenSSL.

DBoW2, g2o, and Sophus are provided in `Thirdparty/` and must be built and installed first.

### Build

```bash
cmake -B build -S . -GNinja -DCMAKE_BUILD_TYPE=Release
ninja -C build
```

### Targets

| Target | Type | Description |
|--------|------|-------------|
| `orb_slam3` | Shared library | Main SLAM library (`libORB_SLAM3.so`) |
| `orb_slam3_offline` | Executable | Unified offline binary for all sensor modes |
| `*_test` | Executables | GoogleTest test binaries |

### Install

```bash
cmake --install build --prefix /usr/local
```

Installs: library to `lib/`, headers to `include/orb_slam3/`, binary to `bin/`, CMake config to `lib/cmake/orb_slam3/`.

---

## Tests

Six GoogleTest suites in `test/`:

- `LoggingUtils_test.cc`
- `Settings_test.cc`
- `MapDrawer_test.cc`
- `Viewer_test.cc`
- `Tracking_test.cc`
- `DatasetRunner_test.cc`

```bash
# Run all tests
ctest --test-dir build

# Run a single test
./build/LoggingUtils_test
```

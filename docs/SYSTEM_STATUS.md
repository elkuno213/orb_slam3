# System Status

**Project:** ORB-SLAM3 C++20 Modernized Fork
**Version:** v1.1.0
**Branch:** `feature/offline-binary-v2`
**Last updated:** 2026-03-02

---

## Completed

- **C++20 standard** — `CMAKE_CXX_STANDARD 20`, `CXX_EXTENSIONS OFF`
- **Modern CMake 3.15+** — target-based dependency management, Ninja generator
- **Unified offline binary** (`orb_slam3_offline`) — replaces 12 legacy per-dataset binaries using Strategy pattern
- **DatasetRunner strategies** — EuRoCRunner, TumRunner, TumViRunner with CLI via Boost.program_options
- **spdlog logging** — replaced all `cout`/`cerr` with structured logging
- **Multi-stage Docker build** — base, deps, dev, builder, runtime, evo stages
- **Docker Compose** — 3 services: runtime, dev, evo
- **Evaluation pipeline** (`verify.py`) — ATE RMSE baseline/verify modes using evo toolkit
- **GoogleTest integration** — 6 test files (LoggingUtils, Settings, MapDrawer, Viewer, Tracking, DatasetRunner)
- **Pangolin v0.9.4** visualization with `--no-viewer` CLI flag for headless runs
- **In-tree thirdparty** — DBoW2, g2o, Sophus
- **Boost.Serialization** — Atlas/Map/KeyFrame/MapPoint persistence
- **Versio-based versioning** — automated changelog and semver
- **clang-format-15** — Google-based style config

## In Progress

- **Phase 0: Mechanical cleanup** — `nullptr`, `#pragma once`, `sleep_for`, spdlog migration (mostly done)
- **Phase 1: CMake modernization** — done
- **Unified binary v2** — merging `DatasetRunners.{h,cc}` into `DatasetRunner.{h,cc}`, simplifying `verify.py` (branch: `feature/offline-binary-v2`)

## Planned (Modernization Roadmap)

| Phase | Description |
|-------|-------------|
| 2 | Smart pointers and RAII — replace raw pointers with `unique_ptr`/`shared_ptr` |
| 3 | Optimizer.cc decomposition — break up monolithic ~5000+ line file |
| 4 | Core/Integration split — pure C++ `core/` vs `ros2/` integration layer |
| 5 | Test expansion — target 80% coverage (currently 6 test files) |
| 6 | C++20 features — concepts, `jthread`, `expected`, `span` |
| 7 | Thread safety — atomics, `shared_mutex`, TSan validation |
| 8 | Naming convention alignment (optional) — migrate Hungarian notation to modern style |

## Known Issues

| Issue | Severity | Mitigation |
|-------|----------|------------|
| Docker build is resource-intensive | Medium | Use `-j4` max (not `-j$(nproc)`) to avoid OOM |
| Hungarian notation inconsistency | Low | New code uses modern naming; legacy code unchanged |
| Optimizer.cc is monolithic (~5000+ lines) | Medium | Planned decomposition in Phase 3 |
| Raw pointer ownership unclear in some classes | Medium | Phase 2 will introduce smart pointers |
| Some camera model files use `.cpp` extension | Low | Legacy; to be unified to `.cc` |

## Next Steps

1. Complete unified binary v2 — merge DatasetRunners, build, test, verify with `verify.py`
2. Begin Phase 2 — smart pointer migration starting with MapPoint and KeyFrame
3. Expand test coverage beyond current 6 test files
4. Profile performance to establish optimization baselines
5. Document public API with Doxygen comments

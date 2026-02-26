# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage: base — system packages shared by all dependency builds                                    #

FROM ubuntu:24.04 AS base

ARG NPROC
ENV DEBIAN_FRONTEND=noninteractive \
    NPROC=${NPROC:-}

# System dependencies: build tools, display libs, codecs, Python, ORB-SLAM3 deps,
# and Mesa utilities (glxinfo, glxgears).
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential                                          \
        ca-certificates                                          \
        cmake                                                    \
        git                                                      \
        ninja-build                                              \
        pkg-config                                               \
        # OpenGL / EGL / Wayland / X11                           \
        libegl1-mesa-dev                                         \
        libepoxy-dev                                             \
        libgl1-mesa-dev                                          \
        libglew-dev                                              \
        libwayland-dev                                           \
        libx11-dev                                               \
        libxcursor-dev                                           \
        libxi-dev                                                \
        libxinerama-dev                                          \
        libxkbcommon-dev                                         \
        libxrandr-dev                                            \
        wayland-protocols                                        \
        # Image / video codecs                                   \
        libavcodec-dev                                           \
        libavformat-dev                                          \
        libavutil-dev                                            \
        libjpeg-dev                                              \
        libpng-dev                                               \
        libswscale-dev                                           \
        libtiff-dev                                              \
        # Python — Pangolin CMake dependency                     \
        python3-dev                                              \
        python3-setuptools                                       \
        python3-wheel                                            \
        # ORB-SLAM3 deps                                         \
        libboost-program-options-dev                             \
        libboost-serialization-dev                               \
        libeigen3-dev                                            \
        libgtest-dev                                             \
        libopencv-dev                                            \
        libspdlog-dev                                            \
        libssl-dev                                               \
        # Mesa / GLX debugging utils                             \
        libglu1-mesa-dev                                         \
        mesa-utils                                               \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /orb-slam3

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Per-dependency build stages — independent stages that can be built in parallel.                  #
# Each installs to /usr/local (the default prefix). The merge stage simply copies /usr/local/      #
# from each stage — redundant base files are harmless and cmake configs need no fixup.             #

# Pangolin — 3D visualization and UI framework used by ORB-SLAM3's map viewer
FROM base AS build-pangolin
RUN git clone --recursive --depth 1 --branch v0.9.4 \
        https://github.com/stevenlovegrove/Pangolin.git Thirdparty/Pangolin
RUN cmake -B Thirdparty/Pangolin/build -S Thirdparty/Pangolin -GNinja \
        -DCMAKE_BUILD_TYPE=Release                                    \
        -DCMAKE_INSTALL_PREFIX=/usr/local                             \
        -DPython_EXECUTABLE="$(which python3)"                        \
        -DBUILD_TESTS=OFF                                             \
        -DBUILD_EXAMPLES=OFF                                          \
    && ninja -C Thirdparty/Pangolin/build -j"${NPROC:-$(nproc)}"      \
    && cmake --install Thirdparty/Pangolin/build

# DBoW2 — bag-of-words library for visual place recognition and loop closure
FROM base AS build-dbow2
COPY Thirdparty/DBoW2/ Thirdparty/DBoW2/
RUN cmake -B Thirdparty/DBoW2/build -S Thirdparty/DBoW2 -GNinja \
        -DCMAKE_BUILD_TYPE=Release                              \
        -DCMAKE_INSTALL_PREFIX=/usr/local                       \
    && ninja -C Thirdparty/DBoW2/build -j"${NPROC:-$(nproc)}"   \
    && cmake --install Thirdparty/DBoW2/build

# g2o — general graph optimization framework for pose-graph and bundle adjustment
FROM base AS build-g2o
COPY Thirdparty/g2o/ Thirdparty/g2o/
RUN cmake -B Thirdparty/g2o/build -S Thirdparty/g2o -GNinja \
        -DCMAKE_BUILD_TYPE=Release                          \
        -DCMAKE_INSTALL_PREFIX=/usr/local                   \
    && ninja -C Thirdparty/g2o/build -j"${NPROC:-$(nproc)}" \
    && cmake --install Thirdparty/g2o/build

# Sophus — header-only C++ Lie group library (SO3/SE3) used for rigid-body transforms
FROM base AS build-sophus
COPY Thirdparty/Sophus/ Thirdparty/Sophus/
RUN cmake -B Thirdparty/Sophus/build -S Thirdparty/Sophus -GNinja \
        -DCMAKE_BUILD_TYPE=Release                                \
        -DCMAKE_INSTALL_PREFIX=/usr/local                         \
        -DBUILD_TESTS=OFF                                         \
        -DBUILD_EXAMPLES=OFF                                      \
    && ninja -C Thirdparty/Sophus/build -j"${NPROC:-$(nproc)}"    \
    && cmake --install Thirdparty/Sophus/build

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage: deps — merge all pre-built dependencies into a single /usr/local prefix                   #

FROM base AS deps

COPY --from=build-pangolin     /usr/local/ /usr/local/
COPY --from=build-dbow2        /usr/local/ /usr/local/
COPY --from=build-g2o          /usr/local/ /usr/local/
COPY --from=build-sophus       /usr/local/ /usr/local/

# Ensure all shared libraries installed to /usr/local/lib are discoverable
RUN ldconfig

# ORB vocabulary — pre-trained visual word dictionary for DBoW2 place recognition
COPY Vocabulary/ Vocabulary/
WORKDIR /orb-slam3/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz
WORKDIR /orb-slam3

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage: dev — development environment with pre-built deps and editor/debug tooling                 #

FROM deps AS dev

RUN apt-get update && apt-get install -y --no-install-recommends \
        clangd                                                   \
        curl                                                     \
        wget                                                     \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage: builder — compile ORB-SLAM3 against the pre-built dependencies                            #

FROM deps AS builder
COPY . .
RUN --mount=type=cache,id=orb-slam3-build,target=/orb-slam3/build \
    cmake -B build -S . -GNinja                                   \
        -DCMAKE_BUILD_TYPE=Release                                \
        -DCMAKE_INSTALL_PREFIX=/usr/local                         \
        -DBUILD_TESTING=OFF                                       \
    && ninja -C build -j"${NPROC:-$(nproc)}"                      \
    && cmake --install build

# Prepare minimal runtime artifacts (only shared libs and binaries, no headers/archives/cmake)
RUN mkdir -p /runtime/lib /runtime/bin                                   \
    && find /usr/local/lib -name '*.so*' -exec cp -a {} /runtime/lib/ \; \
    && cp -a /usr/local/bin/* /runtime/bin/

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage: runtime — minimal image with only binaries, shared libs, and vocabulary                   #

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime-only shared libraries (no -dev headers or build tools).
RUN apt-get update && apt-get install -y --no-install-recommends \
        # OpenGL / EGL / Wayland / X11                           \
        libegl1                                                  \
        libepoxy0                                                \
        libgl1                                                   \
        libglew2.2                                               \
        libwayland-client0                                       \
        libwayland-cursor0                                       \
        libx11-6                                                 \
        libxcursor1                                              \
        libxi6                                                   \
        libxinerama1                                             \
        libxkbcommon0                                            \
        libxrandr2                                               \
        # Image / video codecs                                   \
        libavcodec60                                             \
        libavformat60                                            \
        libavutil58                                              \
        libjpeg8                                                 \
        libpng16-16t64                                           \
        libswscale7                                              \
        libtiff6                                                 \
        # OpenCV runtime modules                                 \
        libopencv-calib3d406t64                                  \
        libopencv-core406t64                                     \
        libopencv-features2d406t64                               \
        libopencv-highgui406t64                                  \
        libopencv-imgcodecs406t64                                \
        libopencv-imgproc406t64                                  \
        libopencv-videoio406t64                                  \
        # Boost, OpenSSL, logging, OpenMP                        \
        libboost-program-options1.83.0                           \
        libboost-serialization1.83.0                             \
        libgomp1                                                 \
        libspdlog1.12                                            \
        libssl3t64                                               \
        # Mesa / GLX debugging utils                             \
        libglu1-mesa                                             \
        mesa-utils                                               \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /orb-slam3

# Only shared libraries and binaries (no headers, static archives, or cmake configs)
COPY --from=builder /runtime/lib/ /usr/local/lib/
COPY --from=builder /runtime/bin/ /usr/local/bin/

# ORB vocabulary
COPY --from=builder /orb-slam3/Vocabulary/ORBvoc.txt Vocabulary/ORBvoc.txt

# Example configuration and calibration files (executables are in /usr/local/bin/)
COPY --from=builder /orb-slam3/Examples/ Examples/

RUN ldconfig

ENTRYPOINT ["/bin/bash"]

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage: evo — evaluation with Python + evo toolkit + SLAM binaries                                #

FROM runtime AS evo

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip git                                  \
    && pip3 install --no-cache-dir --break-system-packages evo   \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT []
CMD ["/bin/bash"]

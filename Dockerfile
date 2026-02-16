# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage 1: deps — system packages and third-party library builds                                   #

FROM ubuntu:22.04 AS deps

ENV DEBIAN_FRONTEND=noninteractive

# System dependencies: build tools, display libs, codecs, Python, ORB-SLAM3 deps,
# Intel RealSense build deps, and Mesa utilities (glxinfo, glxgears).
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
        # Intel RealSense build deps                             \
        libgtk-3-dev                                             \
        libudev-dev                                              \
        libusb-1.0-0-dev                                         \
        # Mesa / GLX debugging utils                             \
        libglu1-mesa-dev                                         \
        mesa-utils                                               \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /orb_slam3

# Intel RealSense SDK — camera interface for D435i / T265 examples.
# Built from source with -DBUILD_WITH_DDS=OFF to avoid the fastcdr/fastrtps dependency whose Ubuntu
# packages lack CMake exported targets.
RUN git clone --depth 1 --branch v2.56.5 \
        https://github.com/IntelRealSense/librealsense.git Thirdparty/librealsense
RUN --mount=type=cache,target=/orb_slam3/Thirdparty/librealsense/build        \
    cmake -B Thirdparty/librealsense/build -S Thirdparty/librealsense -GNinja \
        -DCMAKE_BUILD_TYPE=Release                                            \
        -DCMAKE_INSTALL_PREFIX=/usr/local                                     \
        -DBUILD_EXAMPLES=OFF                                                  \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF                                        \
        -DBUILD_TOOLS=OFF                                                     \
        -DBUILD_UNIT_TESTS=OFF                                                \
        -DBUILD_WITH_DDS=OFF                                                  \
    && ninja -C Thirdparty/librealsense/build -j"$(nproc)"                    \
    && cmake --install Thirdparty/librealsense/build

# Pangolin — 3D visualization and UI framework used by ORB-SLAM3's map viewer #
RUN git clone --recursive --depth 1 --branch v0.9.4 \
        https://github.com/stevenlovegrove/Pangolin.git Thirdparty/Pangolin
RUN --mount=type=cache,target=/orb_slam3/Thirdparty/Pangolin/build    \
    cmake -B Thirdparty/Pangolin/build -S Thirdparty/Pangolin -GNinja \
        -DCMAKE_BUILD_TYPE=Release                                    \
        -DCMAKE_INSTALL_PREFIX=/usr/local                             \
        -DPython_EXECUTABLE="$(which python3)"                        \
        -DBUILD_TESTS=OFF                                             \
        -DBUILD_EXAMPLES=OFF                                          \
    && ninja -C Thirdparty/Pangolin/build -j"$(nproc)"                \
    && cmake --install Thirdparty/Pangolin/build

# DBoW2 — bag-of-words library for visual place recognition and loop closure
COPY Thirdparty/DBoW2/ Thirdparty/DBoW2/
RUN --mount=type=cache,target=/orb_slam3/Thirdparty/DBoW2/build \
    cmake -B Thirdparty/DBoW2/build -S Thirdparty/DBoW2 -GNinja \
        -DCMAKE_BUILD_TYPE=Release                              \
    && ninja -C Thirdparty/DBoW2/build -j"$(nproc)"             \
    && cmake --install Thirdparty/DBoW2/build

# g2o — general graph optimization framework for pose-graph and bundle adjustment
COPY Thirdparty/g2o/ Thirdparty/g2o/
RUN --mount=type=cache,target=/orb_slam3/Thirdparty/g2o/build \
    cmake -B Thirdparty/g2o/build -S Thirdparty/g2o -GNinja   \
        -DCMAKE_BUILD_TYPE=Release                            \
    && ninja -C Thirdparty/g2o/build -j"$(nproc)"             \
    && cmake --install Thirdparty/g2o/build

# Sophus — header-only C++ Lie group library (SO3/SE3) used for rigid-body transforms
COPY Thirdparty/Sophus/ Thirdparty/Sophus/
RUN --mount=type=cache,target=/orb_slam3/Thirdparty/Sophus/build  \
    cmake -B Thirdparty/Sophus/build -S Thirdparty/Sophus -GNinja \
        -DCMAKE_BUILD_TYPE=Release                                \
        -DBUILD_TESTS=OFF                                         \
        -DBUILD_EXAMPLES=OFF                                      \
    && ninja -C Thirdparty/Sophus/build -j"$(nproc)"              \
    && cmake --install Thirdparty/Sophus/build

# Ensure all shared libraries installed to /usr/local/lib are discoverable
RUN ldconfig

# ORB vocabulary — pre-trained visual word dictionary for DBoW2 place recognition
COPY Vocabulary/ Vocabulary/
WORKDIR /orb_slam3/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz
WORKDIR /orb_slam3

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage 2: builder — compile ORB-SLAM3 against the pre-built dependencies                          #

FROM deps AS builder
COPY . .
RUN --mount=type=cache,target=/orb_slam3/build \
    cmake -B build -S . -GNinja                \
        -DCMAKE_BUILD_TYPE=Release             \
        -DCMAKE_INSTALL_PREFIX=/usr/local      \
        -DBUILD_TESTING=OFF                    \
    && ninja -C build -j"$(nproc)"             \
    && cmake --install build

# Prepare minimal runtime artifacts (only shared libs and binaries, no headers/archives/cmake)
RUN mkdir -p /runtime/lib /runtime/bin                                   \
    && find /usr/local/lib -name '*.so*' -exec cp -a {} /runtime/lib/ \; \
    && cp -a /usr/local/bin/* /runtime/bin/

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage 3: runtime — minimal image with only binaries, shared libs, and vocabulary                 #

FROM ubuntu:22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime-only shared libraries (no -dev headers or build tools).
RUN apt-get update && apt-get install -y --no-install-recommends \
        # OpenGL / EGL / Wayland / X11                           \
        libegl1                                                  \
        libepoxy0                                                \
        libgl1                                                   \
        libglew2.2                                               \
        libwayland-client0                                       \
        libx11-6                                                 \
        libxcursor1                                              \
        libxi6                                                   \
        libxinerama1                                             \
        libxkbcommon0                                            \
        libxrandr2                                               \
        # Image / video codecs                                   \
        libavcodec58                                             \
        libavformat58                                            \
        libavutil56                                              \
        libjpeg8                                                 \
        libpng16-16                                              \
        libswscale5                                              \
        libtiff5                                                 \
        # OpenCV runtime modules                                 \
        libopencv-calib3d4.5d                                    \
        libopencv-core4.5d                                       \
        libopencv-features2d4.5d                                 \
        libopencv-highgui4.5d                                    \
        libopencv-imgcodecs4.5d                                  \
        libopencv-imgproc4.5d                                    \
        libopencv-videoio4.5d                                    \
        # Boost, OpenSSL, logging, OpenMP                        \
        libboost-program-options1.74.0                           \
        libboost-serialization1.74.0                             \
        libgomp1                                                 \
        libspdlog1                                               \
        libssl3                                                  \
        # Intel RealSense runtime                                \
        libudev1                                                 \
        libusb-1.0-0                                             \
        # Mesa / GLX debugging utils                             \
        libglu1-mesa                                             \
        mesa-utils                                               \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /orb_slam3

# Only shared libraries and binaries (no headers, static archives, or cmake configs)
COPY --from=builder /runtime/lib/ /usr/local/lib/
COPY --from=builder /runtime/bin/ /usr/local/bin/

# ORB vocabulary
COPY --from=builder /orb_slam3/Vocabulary/ORBvoc.txt Vocabulary/ORBvoc.txt

# Example configuration and calibration files (executables are in /usr/local/bin/)
COPY --from=builder /orb_slam3/Examples/ Examples/

RUN ldconfig

ENTRYPOINT ["/bin/bash"]

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Stage 4: evo — evaluation with Python + evo toolkit + SLAM binaries                             #

FROM runtime AS evo

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip git \
    && pip3 install --no-cache-dir evo \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT []
CMD ["/bin/bash"]

FROM orb-slam3:dev AS builder
WORKDIR /orb_slam3
COPY . .
RUN cmake -B build -S . -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTING=ON \
        -DBUILD_LEGACY_EXAMPLES=OFF
RUN ninja -C build -j"$(nproc)" 2>&1 || true
RUN ninja -C build -j1 orb_slam3_offline 2>&1 || true

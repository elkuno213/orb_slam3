#!/bin/bash
set -e

echo "Installing build tools ..."

apt-get install -y \
  sudo             \
  build-essential  \
  cmake            \
  ninja-build

echo "Installing Pangolin ..."

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
chmod +x ./scripts/install_prerequisites.sh
yes | ./scripts/install_prerequisites.sh all
apt-get install -y   \
  python3-setuptools \
  python3-wheel
cmake -B build                            \
      -S .                                \
      -GNinja                             \
      -DCMAKE_BUILD_TYPE=Release          \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON  \
      -DPython_EXECUTABLE=`which python3` \
      -DBUILD_TESTS=ON
ninja -C build -j 4
ninja -C build -j 4 test
ninja -C build install
cd ..

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
cmake -B build                           \
      -S .                               \
      -GNinja                            \
      -DCMAKE_BUILD_TYPE=Release         \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja -C build -j 4
sudo cmake --install build
cd ../../

echo "Configuring and building Thirdparty/g2o ..."

cd Thirdparty/g2o
cmake -B build                           \
      -S .                               \
      -GNinja                            \
      -DCMAKE_BUILD_TYPE=Release         \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja -C build -j 4
sudo cmake --install build
cd ../../

echo "Configuring and building Thirdparty/Sophus ..."

cd Thirdparty/Sophus
cmake -B build                           \
      -S .                               \
      -GNinja                            \
      -DCMAKE_BUILD_TYPE=Release         \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja -C build -j 4
sudo cmake --install build
cd ../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ../

echo "Configuring and building ORB_SLAM3 ..."

# Install apt dependencies.
apt-get install -y             \
  libeigen3-dev                \
  libopencv-dev                \
  libboost-serialization-dev   \
  libboost-program-options-dev \
  libssl-dev                   \
  libspdlog-dev                \
  libgtest-dev                 \
  lsb-release

# Install librealsense2.
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
# Add the server to the list of repositories
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
apt-get update
# Install the libraries
apt-get install -y librealsense2-dev

cmake -B build                           \
      -S .                               \
      -GNinja                            \
      -DCMAKE_BUILD_TYPE=Release         \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja -C build -j 4

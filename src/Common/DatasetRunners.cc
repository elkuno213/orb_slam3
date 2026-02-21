/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gomez Rodriguez, Jose M.M. Montiel
 * and Juan D. Tardos, University of Zaragoza. Copyright (C) 2014-2016 Raul Mur-Artal, Jose M.M.
 * Montiel and Juan D. Tardos, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Common/DatasetRunners.h"
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include "Common/EuRoC.h"
#include "Common/KITTI.h"
#include "Common/TUM.h"
#include "Common/TUMVI.h"

namespace ORB_SLAM3 {

namespace {

/// Check if image resizing is needed (avoids floating-point equality with 1.0f).
[[nodiscard]] bool needsResize(float image_scale) noexcept {
  return std::abs(image_scale - 1.f) > 1e-6f;
}

} // namespace

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// EuRoCRunner                                                                                    //

EuRoCRunner::EuRoCRunner(const RunConfig& config)
  : _sensor_type([&] {
    if (config.sensor == "mono") {
      return config.inertial ? System::IMU_MONOCULAR : System::MONOCULAR;
    }
    return config.inertial ? System::IMU_STEREO : System::STEREO;
  }())
  , _has_imu(config.inertial)
  , _raw_sequences(config.sequences) {
}

void EuRoCRunner::load() {
  constexpr int kStride = 2; // (sequence_path, timestamps_file) per sequence.
  const auto    n_args  = static_cast<int>(_raw_sequences.size());
  if (n_args % kStride != 0) {
    spdlog::warn(
      "EuRoC: --sequences has {} args, not divisible by {} — trailing args ignored",
      n_args,
      kStride
    );
  }

  const int num_seq = n_args / kStride;
  _sequences.resize(num_seq);

  for (int seq = 0; seq < num_seq; seq++) {
    const std::string& path_seq        = _raw_sequences[2 * seq];
    const std::string& path_timestamps = _raw_sequences[2 * seq + 1];

    const std::string path_cam0 = path_seq + "/mav0/cam0/data";
    const std::string path_cam1 = path_seq + "/mav0/cam1/data";
    const std::string path_imu  = path_seq + "/mav0/imu0/data.csv";

    spdlog::info("Loading images for sequence {}...", seq);
    if (_sensor_type == System::MONOCULAR || _sensor_type == System::IMU_MONOCULAR) {
      EuRoC::LoadMonocularImages(
        path_cam0,
        path_timestamps,
        _sequences[seq].image_filenames,
        _sequences[seq].timestamps
      );
    } else {
      EuRoC::LoadStereoImages(
        path_cam0,
        path_cam1,
        path_timestamps,
        _sequences[seq].image_filenames,
        _sequences[seq].image_right_filenames,
        _sequences[seq].timestamps
      );
    }
    spdlog::info("Images loaded!");

    if (_has_imu) {
      spdlog::info("Loading IMU for sequence {}...", seq);
      EuRoC::LoadIMU(
        path_imu,
        _sequences[seq].imu_timestamps,
        _sequences[seq].acc,
        _sequences[seq].gyro
      );
      spdlog::info("IMU data loaded!");

      if (_sequences[seq].imu_timestamps.empty()) {
        throw std::runtime_error("Failed to load IMU for sequence " + std::to_string(seq));
      }

      // Find first IMU measurement after first camera timestamp.
      auto& sd = _sequences[seq];
      while (sd.first_imu < std::ssize(sd.imu_timestamps)
             && sd.imu_timestamps[sd.first_imu] <= sd.timestamps[0]) {
        sd.first_imu++;
      }
      if (sd.first_imu > 0) {
        sd.first_imu--;
      }
    }

    if (_sequences[seq].image_filenames.empty()) {
      throw std::runtime_error("Failed to load images for sequence " + std::to_string(seq));
    }
  }
}

int EuRoCRunner::numSequences() const {
  return static_cast<int>(_sequences.size());
}

int EuRoCRunner::numImages(int seq) const {
  return static_cast<int>(_sequences[seq].image_filenames.size());
}

double EuRoCRunner::readFrame(
  int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
) {
  im = cv::imread(_sequences[seq].image_filenames[ni], cv::IMREAD_UNCHANGED);
  if (im.empty()) {
    throw std::runtime_error("Failed to load image: " + _sequences[seq].image_filenames[ni]);
  }

  if (_sensor_type == System::STEREO || _sensor_type == System::IMU_STEREO) {
    im_right = cv::imread(_sequences[seq].image_right_filenames[ni], cv::IMREAD_UNCHANGED);
    if (im_right.empty()) {
      throw std::runtime_error(
        "Failed to load right image: " + _sequences[seq].image_right_filenames[ni]
      );
    }
  }

  if (needsResize(image_scale)) {
    const int width  = static_cast<int>(im.cols * image_scale);
    const int height = static_cast<int>(im.rows * image_scale);
    cv::resize(im, im, cv::Size(width, height));
    if (!im_right.empty()) {
      cv::resize(im_right, im_right, cv::Size(width, height));
    }
  }

  return _sequences[seq].timestamps[ni];
}

std::vector<IMU::Point> EuRoCRunner::collectImu(int seq, int ni) {
  std::vector<IMU::Point> imu_meas;
  if (!_has_imu || ni == 0) {
    return imu_meas;
  }

  auto&      sd    = _sequences[seq];
  const auto n_imu = std::ssize(sd.imu_timestamps);
  while (sd.first_imu < n_imu && sd.imu_timestamps[sd.first_imu] <= sd.timestamps[ni]) {
    imu_meas.emplace_back(
      sd.acc[sd.first_imu].x,
      sd.acc[sd.first_imu].y,
      sd.acc[sd.first_imu].z,
      sd.gyro[sd.first_imu].x,
      sd.gyro[sd.first_imu].y,
      sd.gyro[sd.first_imu].z,
      sd.imu_timestamps[sd.first_imu]
    );
    sd.first_imu++;
  }
  return imu_meas;
}

System::eSensor EuRoCRunner::sensorType() const noexcept {
  return _sensor_type;
}

TrajectoryFormat EuRoCRunner::trajectoryFormat() const noexcept {
  return TrajectoryFormat::kEuRoC;
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// KittiRunner                                                                                    //

KittiRunner::KittiRunner(const RunConfig& config)
  : _sensor_type(config.sensor == "mono" ? System::MONOCULAR : System::STEREO)
  , _sequence_dir(config.sequence_dir) {
}

void KittiRunner::load() {
  _sequences.resize(1);
  auto& sd = _sequences[0];

  spdlog::info("Loading images...");
  if (_sensor_type == System::MONOCULAR) {
    KITTI::LoadMonocularImages(_sequence_dir, sd.image_filenames, sd.timestamps);
  } else {
    KITTI::LoadStereoImages(
      _sequence_dir,
      sd.image_filenames,
      sd.image_right_filenames,
      sd.timestamps
    );
  }
  spdlog::info("Images loaded!");

  if (sd.image_filenames.empty()) {
    throw std::runtime_error("Failed to load images from " + _sequence_dir);
  }
}

int KittiRunner::numSequences() const {
  return 1;
}

int KittiRunner::numImages(int seq) const {
  return static_cast<int>(_sequences[seq].image_filenames.size());
}

double KittiRunner::readFrame(
  int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
) {
  im = cv::imread(_sequences[seq].image_filenames[ni], cv::IMREAD_UNCHANGED);
  if (im.empty()) {
    throw std::runtime_error("Failed to load image: " + _sequences[seq].image_filenames[ni]);
  }

  if (_sensor_type == System::STEREO) {
    im_right = cv::imread(_sequences[seq].image_right_filenames[ni], cv::IMREAD_UNCHANGED);
    if (im_right.empty()) {
      throw std::runtime_error(
        "Failed to load right image: " + _sequences[seq].image_right_filenames[ni]
      );
    }
  }

  if (needsResize(image_scale)) {
    const int width  = static_cast<int>(im.cols * image_scale);
    const int height = static_cast<int>(im.rows * image_scale);
    cv::resize(im, im, cv::Size(width, height));
    if (!im_right.empty()) {
      cv::resize(im_right, im_right, cv::Size(width, height));
    }
  }

  return _sequences[seq].timestamps[ni];
}

std::vector<IMU::Point> KittiRunner::collectImu(int /*seq*/, int /*ni*/) {
  return {};
}

System::eSensor KittiRunner::sensorType() const noexcept {
  return _sensor_type;
}

TrajectoryFormat KittiRunner::trajectoryFormat() const noexcept {
  return (_sensor_type == System::MONOCULAR) ? TrajectoryFormat::kTUM : TrajectoryFormat::kKITTI;
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// TumRunner                                                                                      //

TumRunner::TumRunner(const RunConfig& config)
  : _sensor_type(config.sensor == "mono" ? System::MONOCULAR : System::RGBD)
  , _sequence_dir(config.sequence_dir)
  , _association_file(config.association_file) {
}

void TumRunner::load() {
  _sequences.resize(1);
  auto& sd = _sequences[0];

  spdlog::info("Loading images...");
  if (_sensor_type == System::MONOCULAR) {
    const std::string rgb_file = _sequence_dir + "/rgb.txt";
    TUM::LoadMonocularImages(rgb_file, sd.image_filenames, sd.timestamps);
  } else {
    TUM::LoadRGBDImages(_association_file, sd.image_filenames, sd.depth_filenames, sd.timestamps);
    if (sd.image_filenames.size() != sd.depth_filenames.size()) {
      throw std::runtime_error("Mismatch between RGB and depth image counts");
    }
  }
  spdlog::info("Images loaded!");

  if (sd.image_filenames.empty()) {
    throw std::runtime_error("Failed to load images from " + _sequence_dir);
  }
}

int TumRunner::numSequences() const {
  return 1;
}

int TumRunner::numImages(int seq) const {
  return static_cast<int>(_sequences[seq].image_filenames.size());
}

double TumRunner::readFrame(
  int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
) {
  // TUM loaders return relative paths — prepend sequence_dir
  im = cv::imread(_sequence_dir + "/" + _sequences[seq].image_filenames[ni], cv::IMREAD_UNCHANGED);
  if (im.empty()) {
    throw std::runtime_error(
      "Failed to load image: " + _sequence_dir + "/" + _sequences[seq].image_filenames[ni]
    );
  }

  if (_sensor_type == System::RGBD) {
    depth
      = cv::imread(_sequence_dir + "/" + _sequences[seq].depth_filenames[ni], cv::IMREAD_UNCHANGED);
    if (depth.empty()) {
      throw std::runtime_error(
        "Failed to load depth: " + _sequence_dir + "/" + _sequences[seq].depth_filenames[ni]
      );
    }
  }

  if (needsResize(image_scale)) {
    const int width  = static_cast<int>(im.cols * image_scale);
    const int height = static_cast<int>(im.rows * image_scale);
    cv::resize(im, im, cv::Size(width, height));
    if (!depth.empty()) {
      cv::resize(depth, depth, cv::Size(width, height));
    }
  }

  return _sequences[seq].timestamps[ni];
}

std::vector<IMU::Point> TumRunner::collectImu(int /*seq*/, int /*ni*/) {
  return {};
}

System::eSensor TumRunner::sensorType() const noexcept {
  return _sensor_type;
}

TrajectoryFormat TumRunner::trajectoryFormat() const noexcept {
  return TrajectoryFormat::kTUM;
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// TumViRunner                                                                                    //

TumViRunner::TumViRunner(const RunConfig& config)
  : _sensor_type([&] {
    if (config.sensor == "mono") {
      return config.inertial ? System::IMU_MONOCULAR : System::MONOCULAR;
    }
    return config.inertial ? System::IMU_STEREO : System::STEREO;
  }())
  , _has_imu(config.inertial)
  , _is_stereo(config.sensor == "stereo")
  , _raw_sequences(config.sequences)
  , _output_dir(config.output_dir) {
}

void TumViRunner::load() {
  // Determine stride based on (sensor x inertial)
  // mono: stride 2 (image_dir, times_file)
  // stereo: stride 3 (left_dir, right_dir, times_file)
  // mono-inertial: stride 3 (image_dir, times_file, imu_file)
  // stereo-inertial: stride 4 (left_dir, right_dir, times_file, imu_file)
  int stride = 2;
  if (_is_stereo) {
    stride++;
  }
  if (_has_imu) {
    stride++;
  }

  const auto n_args = static_cast<int>(_raw_sequences.size());
  if (n_args % stride != 0) {
    spdlog::warn(
      "TUM-VI: --sequences has {} args, not divisible by {} — trailing args ignored",
      n_args,
      stride
    );
  }

  const int num_seq = n_args / stride;
  _sequences.resize(num_seq);

  for (int seq = 0; seq < num_seq; seq++) {
    const int idx = seq * stride;

    std::string path_images;
    std::string path_right;
    std::string path_timestamps;
    std::string path_imu;

    if (_is_stereo && _has_imu) {
      // stride 4: left_dir, right_dir, times_file, imu_file
      path_images     = _raw_sequences[idx];
      path_right      = _raw_sequences[idx + 1];
      path_timestamps = _raw_sequences[idx + 2];
      path_imu        = _raw_sequences[idx + 3];
    } else if (_is_stereo) {
      // stride 3: left_dir, right_dir, times_file
      path_images     = _raw_sequences[idx];
      path_right      = _raw_sequences[idx + 1];
      path_timestamps = _raw_sequences[idx + 2];
    } else if (_has_imu) {
      // stride 3: image_dir, times_file, imu_file
      path_images     = _raw_sequences[idx];
      path_timestamps = _raw_sequences[idx + 1];
      path_imu        = _raw_sequences[idx + 2];
    } else {
      // stride 2: image_dir, times_file
      path_images     = _raw_sequences[idx];
      path_timestamps = _raw_sequences[idx + 1];
    }

    spdlog::info("Loading images for sequence {}...", seq);
    if (_is_stereo) {
      TUMVI::LoadStereoImages(
        path_images,
        path_right,
        path_timestamps,
        _sequences[seq].image_filenames,
        _sequences[seq].image_right_filenames,
        _sequences[seq].timestamps
      );
    } else {
      TUMVI::LoadMonocularImages(
        path_images,
        path_timestamps,
        _sequences[seq].image_filenames,
        _sequences[seq].timestamps
      );
    }
    spdlog::info("Images loaded!");

    if (_has_imu) {
      spdlog::info("Loading IMU for sequence {}...", seq);
      TUMVI::LoadIMU(
        path_imu,
        _sequences[seq].imu_timestamps,
        _sequences[seq].acc,
        _sequences[seq].gyro
      );
      spdlog::info("IMU data loaded!");

      if (_sequences[seq].imu_timestamps.empty()) {
        throw std::runtime_error("Failed to load IMU for sequence " + std::to_string(seq));
      }

      auto& sd = _sequences[seq];
      while (sd.first_imu < std::ssize(sd.imu_timestamps)
             && sd.imu_timestamps[sd.first_imu] <= sd.timestamps[0]) {
        sd.first_imu++;
      }
      if (sd.first_imu > 0) {
        sd.first_imu--;
      }
    }

    if (_sequences[seq].image_filenames.empty()) {
      throw std::runtime_error("Failed to load images for sequence " + std::to_string(seq));
    }
  }
}

int TumViRunner::numSequences() const {
  return static_cast<int>(_sequences.size());
}

int TumViRunner::numImages(int seq) const {
  return static_cast<int>(_sequences[seq].image_filenames.size());
}

double TumViRunner::readFrame(
  int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
) {
  im = cv::imread(_sequences[seq].image_filenames[ni], cv::IMREAD_GRAYSCALE);
  if (im.empty()) {
    throw std::runtime_error("Failed to load image: " + _sequences[seq].image_filenames[ni]);
  }

  if (_is_stereo) {
    im_right = cv::imread(_sequences[seq].image_right_filenames[ni], cv::IMREAD_GRAYSCALE);
    if (im_right.empty()) {
      throw std::runtime_error(
        "Failed to load right image: " + _sequences[seq].image_right_filenames[ni]
      );
    }
  }

  if (needsResize(image_scale)) {
    const int width  = static_cast<int>(im.cols * image_scale);
    const int height = static_cast<int>(im.rows * image_scale);
    cv::resize(im, im, cv::Size(width, height));
    if (!im_right.empty()) {
      cv::resize(im_right, im_right, cv::Size(width, height));
    }
  }

  return _sequences[seq].timestamps[ni];
}

std::vector<IMU::Point> TumViRunner::collectImu(int seq, int ni) {
  std::vector<IMU::Point> imu_meas;
  if (!_has_imu || ni == 0) {
    return imu_meas;
  }

  auto&      sd    = _sequences[seq];
  const auto n_imu = std::ssize(sd.imu_timestamps);
  while (sd.first_imu < n_imu && sd.imu_timestamps[sd.first_imu] <= sd.timestamps[ni]) {
    imu_meas.emplace_back(
      sd.acc[sd.first_imu].x,
      sd.acc[sd.first_imu].y,
      sd.acc[sd.first_imu].z,
      sd.gyro[sd.first_imu].x,
      sd.gyro[sd.first_imu].y,
      sd.gyro[sd.first_imu].z,
      sd.imu_timestamps[sd.first_imu]
    );
    sd.first_imu++;
  }
  return imu_meas;
}

System::eSensor TumViRunner::sensorType() const noexcept {
  return _sensor_type;
}

TrajectoryFormat TumViRunner::trajectoryFormat() const noexcept {
  return TrajectoryFormat::kEuRoC;
}

bool TumViRunner::useClahe() const noexcept {
  return true;
}

int TumViRunner::imreadMode() const noexcept {
  return cv::IMREAD_GRAYSCALE;
}

std::string TumViRunner::sequenceParam() const noexcept {
  return _output_dir;
}

} // namespace ORB_SLAM3

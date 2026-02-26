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

#include "Common/TumViRunner.h"
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace ORB_SLAM3 {

namespace {

/// Load monocular image paths and timestamps from a mav0/cam0 CSV file.
// TODO(VuHoi): nearly identical to EuRoCRunner — extract into shared CSV loader utility.
void loadMonocularImages(
  const fs::path&           image_path,
  const fs::path&           csv_path,
  std::vector<std::string>& filenames,
  std::vector<double>&      timestamps
) {
  std::ifstream file(csv_path);
  // TODO(VuHoi): check file.is_open() and throw on failure.
  // TODO(VuHoi): replace hardcoded 5000 with a named constant or remove.
  timestamps.reserve(5000);
  filenames.reserve(5000);
  std::string s;
  while (std::getline(file, s)) {
    if (!s.empty() && s.back() == '\r') {
      s.pop_back();
    }

    if (s.empty() || s[0] == '#') {
      continue;
    }

    std::size_t pos = s.find(',');
    std::string ts  = s.substr(0, pos);
    std::string img = (pos != std::string::npos) ? s.substr(pos + 1) : ts + ".png";

    filenames.push_back((image_path / img).string());
    double t = std::stod(ts);
    timestamps.push_back(t * 1e-9);
  }
}

/// Load stereo image paths and timestamps from a mav0/cam0 CSV file.
void loadStereoImages(
  const fs::path&           left_path,
  const fs::path&           right_path,
  const fs::path&           csv_path,
  std::vector<std::string>& left_filenames,
  std::vector<std::string>& right_filenames,
  std::vector<double>&      timestamps
) {
  std::ifstream file(csv_path);
  timestamps.reserve(5000);
  left_filenames.reserve(5000);
  right_filenames.reserve(5000);
  std::string s;
  while (std::getline(file, s)) {
    if (!s.empty() && s.back() == '\r') {
      s.pop_back();
    }

    if (s.empty() || s[0] == '#') {
      continue;
    }

    std::size_t pos = s.find(',');
    std::string ts  = s.substr(0, pos);
    std::string img = (pos != std::string::npos) ? s.substr(pos + 1) : ts + ".png";

    left_filenames.push_back((left_path / img).string());
    right_filenames.push_back((right_path / img).string());
    double t = std::stod(ts);
    timestamps.push_back(t * 1e-9);
  }
}

/// Load IMU data (timestamp, gyro, accelerometer) from a mav0/imu0 CSV file.
void loadImu(
  const fs::path&           imu_path,
  std::vector<double>&      timestamps,
  std::vector<cv::Point3f>& acc,
  std::vector<cv::Point3f>& gyro
) {
  std::ifstream file(imu_path);
  timestamps.reserve(5000);
  acc.reserve(5000);
  gyro.reserve(5000);

  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    if (line.empty() || line[0] == '#') {
      continue;
    }

    // TODO(VuHoi): use std::istringstream or std::from_chars instead of s.erase() — current
    //       approach is O(n^2) per line and count can overflow data[7] on malformed CSV.
    std::string s = line;
    std::string item;
    std::size_t pos = 0;
    double      data[7];
    int         count = 0;
    while ((pos = s.find(',')) != std::string::npos) {
      item          = s.substr(0, pos);
      data[count++] = std::stod(item);
      s.erase(0, pos + 1);
    }
    item    = s.substr(0, pos);
    data[6] = std::stod(item);

    timestamps.push_back(data[0] * 1e-9);
    acc.push_back(cv::Point3f(data[4], data[5], data[6]));
    gyro.push_back(cv::Point3f(data[1], data[2], data[3]));
  }
}

} // namespace

TumViRunner::TumViRunner(const RunConfig& config)
  : _sensor(config.sensor)
  , _has_imu(_sensor == System::IMU_MONOCULAR || _sensor == System::IMU_STEREO)
  , _is_stereo(_sensor == System::STEREO || _sensor == System::IMU_STEREO)
  , _data_dirs(config.data_dirs)
  , _output_dir(config.output_dir) {
}

void TumViRunner::load() {
  const std::size_t num_seq = _data_dirs.size();
  _sequences.resize(num_seq);

  for (std::size_t i = 0; i < num_seq; ++i) {
    auto& sequence = _sequences[i];
    auto& root     = _data_dirs[i];

    const fs::path cam0_data = root / "mav0" / "cam0" / "data";
    const fs::path cam1_data = root / "mav0" / "cam1" / "data";
    const fs::path cam0_csv  = root / "mav0" / "cam0" / "data.csv";
    const fs::path imu_csv   = root / "mav0" / "imu0" / "data.csv";

    if (_is_stereo) {
      loadStereoImages(
        cam0_data,
        cam1_data,
        cam0_csv,
        sequence.img_left_filenames,
        sequence.img_right_filenames,
        sequence.img_timestamps
      );
    } else {
      loadMonocularImages(
        cam0_data,
        cam0_csv,
        sequence.img_left_filenames,
        sequence.img_timestamps
      );
    }

    if (sequence.img_left_filenames.empty()) {
      throw std::runtime_error(
        "No images found for sequence " + std::to_string(i) + ": " + root.string()
      );
    }

    if (_has_imu) {
      loadImu(imu_csv, sequence.imu_timestamps, sequence.acc, sequence.gyro);
      if (sequence.imu_timestamps.empty()) {
        throw std::runtime_error(
          "No IMU data found for sequence " + std::to_string(i) + ": " + root.string()
        );
      }
      syncImu(sequence);
    }

    spdlog::info(
      "TumVI seq {}: {} images, {} IMU samples",
      i,
      sequence.img_left_filenames.size(),
      sequence.imu_timestamps.size()
    );
  }
}

System::eSensor TumViRunner::sensor() const noexcept {
  return _sensor;
}

bool TumViRunner::useClahe() const noexcept {
  return true;
}

int TumViRunner::imreadMode() const noexcept {
  return cv::IMREAD_GRAYSCALE;
}

std::string TumViRunner::param() const noexcept {
  return _output_dir.string();
}

} // namespace ORB_SLAM3

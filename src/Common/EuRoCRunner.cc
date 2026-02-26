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

#include "Common/EuRoCRunner.h"
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace {

/// Load monocular image paths and timestamps from a mav0/cam0 CSV file.
// TODO(VuHoi): loadMonocularImages, loadStereoImages, and loadImu are nearly identical
//       to TumViRunner.cc — extract into a shared CSV loader utility.
void loadMonocularImages(
  const fs::path&           image_path,
  const fs::path&           times_path,
  std::vector<std::string>& filenames,
  std::vector<double>&      timestamps
) {
  std::ifstream file(times_path);
  // TODO(VuHoi): check file.is_open() and throw on failure (TumRunner already does this).
  // TODO(VuHoi): replace hardcoded 5000 with a named constant or remove (vectors grow fine without
  // it).
  timestamps.reserve(5000);
  filenames.reserve(5000);
  std::string line;
  while (std::getline(file, line)) {
    // EuRoC CSVs may have Windows line endings (\r\n) depending on the download tool.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // EuRoC CSV format: "timestamp_ns,image_filename" (comma-separated, not space).
    // Old code used stringstream >> which splits on whitespace — wrong for CSV.
    std::size_t pos = line.find(',');
    std::string ts  = line.substr(0, pos);
    std::string img = (pos != std::string::npos) ? line.substr(pos + 1) : ts + ".png";

    filenames.push_back((image_path / img).string());
    timestamps.push_back(std::stod(ts) * 1e-9);
  }
}

/// Load stereo image paths and timestamps from a mav0/cam0 CSV file.
void loadStereoImages(
  const fs::path&           left_path,
  const fs::path&           right_path,
  const fs::path&           times_path,
  std::vector<std::string>& left_filenames,
  std::vector<std::string>& right_filenames,
  std::vector<double>&      timestamps
) {
  std::ifstream file(times_path);
  timestamps.reserve(5000);
  left_filenames.reserve(5000);
  right_filenames.reserve(5000);
  std::string line;
  while (std::getline(file, line)) {
    // Strip Windows \r (see loadMonocularImages comment).
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // CSV column parsing (see loadMonocularImages comment).
    std::size_t pos = line.find(',');
    std::string ts  = line.substr(0, pos);
    std::string img = (pos != std::string::npos) ? line.substr(pos + 1) : ts + ".png";

    left_filenames.push_back((left_path / img).string());
    right_filenames.push_back((right_path / img).string());
    timestamps.push_back(std::stod(ts) * 1e-9);
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
    // Strip Windows \r (see loadMonocularImages comment).
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // EuRoC IMU CSV: "timestamp_ns,gx,gy,gz,ax,ay,az" (7 comma-separated fields).
    // TODO(VuHoi): use std::istringstream or std::from_chars instead of line.erase() — current
    //       approach is O(n^2) per line and count can overflow data[7] on malformed CSV.
    double      data[7];
    int         count = 0;
    std::size_t pos   = 0;
    while ((pos = line.find(',')) != std::string::npos) {
      data[count++] = std::stod(line.substr(0, pos));
      line.erase(0, pos + 1);
    }
    data[6] = std::stod(line);

    timestamps.push_back(data[0] * 1e-9);
    acc.push_back(cv::Point3f(data[4], data[5], data[6]));
    gyro.push_back(cv::Point3f(data[1], data[2], data[3]));
  }
}

} // anonymous namespace

namespace ORB_SLAM3 {

EuRoCRunner::EuRoCRunner(const RunConfig& config)
  : _sensor(config.sensor)
  , _inertial(config.sensor == System::IMU_MONOCULAR || config.sensor == System::IMU_STEREO)
  , _data_dirs(config.data_dirs) {
}

void EuRoCRunner::load() {
  bool is_stereo = _sensor == System::STEREO || _sensor == System::IMU_STEREO;

  _sequences.resize(_data_dirs.size());

  for (std::size_t i = 0; i < _data_dirs.size(); ++i) {
    const auto& root     = _data_dirs[i];
    auto&       sequence = _sequences[i];

    const fs::path cam0_data = root / "mav0" / "cam0" / "data";
    const fs::path cam1_data = root / "mav0" / "cam1" / "data";
    const fs::path cam0_csv  = root / "mav0" / "cam0" / "data.csv";
    const fs::path imu_csv   = root / "mav0" / "imu0" / "data.csv";

    if (is_stereo) {
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
      throw std::runtime_error("EuRoCRunner: no images loaded from " + root.string());
    }

    spdlog::info("EuRoC seq {}: loaded {} images", i, sequence.img_left_filenames.size());

    if (_inertial) {
      loadImu(imu_csv, sequence.imu_timestamps, sequence.acc, sequence.gyro);
      if (sequence.imu_timestamps.empty()) {
        throw std::runtime_error("EuRoCRunner: no IMU data loaded from " + imu_csv.string());
      }
      spdlog::info("EuRoC seq {}: loaded {} IMU samples", i, sequence.imu_timestamps.size());
      syncImu(sequence);
    }
  }
}

System::eSensor EuRoCRunner::sensor() const noexcept {
  return _sensor;
}

} // namespace ORB_SLAM3

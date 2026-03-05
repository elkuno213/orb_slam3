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

#include "Common/DatasetRunner.h"
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include "Common/EuRoCRunner.h"
#include "Common/TumRunner.h"
#include "Common/TumViRunner.h"

namespace fs = std::filesystem;
namespace po = boost::program_options;

namespace ORB_SLAM3 {

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Base class implementations                                                                     //

std::size_t DatasetRunner::numSequences() const {
  return _sequences.size();
}

std::size_t DatasetRunner::numFrames(std::size_t seq) const {
  return _sequences[seq].img_left_filenames.size();
}

double DatasetRunner::readFrame(
  std::size_t seq,
  std::size_t frame,
  float       img_scale,
  cv::Mat&    img_left,
  cv::Mat&    img_right,
  cv::Mat&    depth
) {
  auto& sequence = _sequences[seq];

  // Load left/mono/RGB image.
  img_left = cv::imread(sequence.img_left_filenames[frame], imreadMode());
  if (img_left.empty()) {
    throw std::runtime_error("Failed to load image: " + sequence.img_left_filenames[frame]);
  }

  // Load right stereo image if available.
  if (!sequence.img_right_filenames.empty()) {
    img_right = cv::imread(sequence.img_right_filenames[frame], imreadMode());
    if (img_right.empty()) {
      throw std::runtime_error(
        "Failed to load right image: " + sequence.img_right_filenames[frame]
      );
    }
  }

  // Load depth image if available (always IMREAD_UNCHANGED for depth).
  if (!sequence.depth_filenames.empty()) {
    depth = cv::imread(sequence.depth_filenames[frame], cv::IMREAD_UNCHANGED);
    if (depth.empty()) {
      throw std::runtime_error("Failed to load depth image: " + sequence.depth_filenames[frame]);
    }
  }

  // Resize if needed.
  if (std::abs(img_scale - 1.f) > 1e-6f) {
    const int width  = static_cast<int>(img_left.cols * img_scale);
    const int height = static_cast<int>(img_left.rows * img_scale);
    cv::resize(img_left, img_left, cv::Size(width, height));
    if (!img_right.empty()) {
      cv::resize(img_right, img_right, cv::Size(width, height));
    }
    if (!depth.empty()) {
      cv::resize(depth, depth, cv::Size(width, height));
    }
  }

  return sequence.img_timestamps[frame];
}

std::vector<IMU::Point> DatasetRunner::readIMU(std::size_t seq, std::size_t frame) {
  std::vector<IMU::Point> result;
  if (seq >= _sequences.size()) {
    return result;
  }
  auto& sequence = _sequences[seq];

  if (sequence.imu_timestamps.empty() || frame == 0) {
    return result;
  }

  while (sequence.first_imu < sequence.imu_timestamps.size()
         && sequence.imu_timestamps[sequence.first_imu] <= sequence.img_timestamps[frame]) {
    result.emplace_back(
      sequence.acc[sequence.first_imu].x,
      sequence.acc[sequence.first_imu].y,
      sequence.acc[sequence.first_imu].z,
      sequence.gyro[sequence.first_imu].x,
      sequence.gyro[sequence.first_imu].y,
      sequence.gyro[sequence.first_imu].z,
      sequence.imu_timestamps[sequence.first_imu]
    );
    sequence.first_imu++;
  }
  return result;
}

void DatasetRunner::syncImu(Sequence& seq) {
  while (seq.first_imu < seq.imu_timestamps.size()
         && seq.imu_timestamps[seq.first_imu] <= seq.img_timestamps[0]) {
    seq.first_imu++;
  }
  if (seq.first_imu > 0) {
    seq.first_imu--;
  }
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Factory                                                                                        //

std::unique_ptr<DatasetRunner> createDatasetRunner(const RunConfig& config) {
  switch (config.dataset) {
    case DatasetType::EuRoC:
      return std::make_unique<EuRoCRunner>(config);
    case DatasetType::TUM:
      return std::make_unique<TumRunner>(config);
    case DatasetType::TumVI:
      return std::make_unique<TumViRunner>(config);
  }
  throw std::invalid_argument("Unknown dataset type");
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// CLI Parser                                                                                     //

namespace {

/// @brief Map string to DatasetType enum.
[[nodiscard]] DatasetType parseDatasetType(const std::string& s) {
  if (s == "euroc") {
    return DatasetType::EuRoC;
  }
  if (s == "tum") {
    return DatasetType::TUM;
  }
  if (s == "tumvi") {
    return DatasetType::TumVI;
  }
  throw po::error("Unknown dataset: " + s + " (euroc|tum|tumvi)");
}

/// @brief Map string sensor + inertial flag to System::eSensor.
[[nodiscard]] System::eSensor parseSensor(const std::string& s, bool inertial) {
  if (s == "mono") {
    return inertial ? System::IMU_MONOCULAR : System::MONOCULAR;
  }
  if (s == "stereo") {
    return inertial ? System::IMU_STEREO : System::STEREO;
  }
  if (s == "rgbd") {
    if (inertial) {
      throw po::error("rgbd does not support inertial mode");
    }
    return System::RGBD;
  }
  throw po::error("Unknown sensor: " + s + " (mono|stereo|rgbd)");
}

/// @brief Validate dataset-sensor compatibility.
void validateConfig(const RunConfig& config) {
  switch (config.dataset) {
    case DatasetType::EuRoC:
      if (config.sensor != System::MONOCULAR && config.sensor != System::IMU_MONOCULAR
          && config.sensor != System::STEREO && config.sensor != System::IMU_STEREO) {
        throw po::error("euroc only supports mono and stereo sensors (with optional inertial)");
      }
      break;
    case DatasetType::TUM:
      if (config.sensor != System::MONOCULAR && config.sensor != System::RGBD) {
        throw po::error("tum only supports mono and rgbd sensors (no inertial)");
      }
      break;
    case DatasetType::TumVI:
      if (config.sensor != System::MONOCULAR && config.sensor != System::IMU_MONOCULAR
          && config.sensor != System::STEREO && config.sensor != System::IMU_STEREO) {
        throw po::error("tumvi only supports mono and stereo sensors (with optional inertial)");
      }
      break;
  }
}

} // namespace

bool parseArgs(int argc, char** argv, RunConfig& config) {
  std::string              dataset_str;
  std::string              sensor_str;
  bool                     inertial = false;
  std::vector<std::string> data_dirs_str;
  std::string              vocabulary_str;
  std::string              settings_str;
  std::string              output_str;

  po::options_description desc("orb_slam3_offline options");
  // clang-format off
  desc.add_options()
    ("help,h",           "Show help message")
    ("dataset",          po::value<std::string>(&dataset_str)->required(),
                         "Dataset type: euroc|tum|tumvi")
    ("sensor",           po::value<std::string>(&sensor_str)->required(),
                         "Sensor type: mono|stereo|rgbd")
    ("inertial",         po::bool_switch(&inertial),
                         "Enable inertial mode (euroc, tumvi only)")
    ("data",             po::value<std::vector<std::string>>(&data_dirs_str)->multitoken()
                           ->required(),
                         "Dataset root directories")
    ("vocabulary-file",  po::value<std::string>(&vocabulary_str)->required(),
                         "Path to vocabulary text file")
    ("settings-file",    po::value<std::string>(&settings_str)->required(),
                         "Path to settings yaml file")
    ("output-dir",       po::value<std::string>(&output_str)->default_value("/tmp"),
                         "Path to output directory")
    ("no-viewer",        "Disable the Pangolin viewer");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    std::ostringstream oss;
    oss << desc;
    spdlog::info("\n{}", oss.str());
    return false;
  }

  po::notify(vm);

  // Convert string args to typed config fields.
  config.dataset = parseDatasetType(dataset_str);
  config.sensor  = parseSensor(sensor_str, inertial);

  config.data_dirs.assign(data_dirs_str.begin(), data_dirs_str.end());
  config.vocabulary_file = vocabulary_str;
  config.settings_file   = settings_str;
  config.output_dir      = output_str;
  config.use_viewer      = (vm.count("no-viewer") == 0);

  // Validate the configuration.
  validateConfig(config);

  // Validate file existence.
  if (!fs::is_regular_file(config.vocabulary_file)) {
    throw po::error("Vocabulary path is not a file: " + config.vocabulary_file.string());
  }
  if (!fs::is_regular_file(config.settings_file)) {
    throw po::error("Settings path is not a file: " + config.settings_file.string());
  }
  if (!fs::is_directory(config.output_dir)) {
    throw po::error("Output directory does NOT exist: " + config.output_dir.string());
  }

  return true;
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Trajectory Saver                                                                               //

void saveTrajectories(
  System& slam, const fs::path& output_dir, DatasetType dataset, System::eSensor sensor
) {
  using enum DatasetType;
  const auto camera   = (output_dir / "CameraTrajectory.txt").string();
  const auto keyframe = (output_dir / "KeyFrameTrajectory.txt").string();

  switch (dataset) {
    case EuRoC:
    case TumVI:
      slam.SaveTrajectoryEuRoC(camera);
      slam.SaveKeyFrameTrajectoryEuRoC(keyframe);
      break;
    case TUM:
      if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR) {
        slam.SaveKeyFrameTrajectoryTUM(keyframe);
      } else {
        slam.SaveTrajectoryTUM(camera);
        slam.SaveKeyFrameTrajectoryTUM(keyframe);
      }
      break;
  }
}

} // namespace ORB_SLAM3

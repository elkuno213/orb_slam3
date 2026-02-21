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
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include "Common/DatasetRunners.h"

namespace fs = std::filesystem;
namespace po = boost::program_options;

namespace ORB_SLAM3 {

// ─────────────────────────────────────────────────────────────────────────────
// Factory
// ─────────────────────────────────────────────────────────────────────────────

std::unique_ptr<DatasetRunner> createDatasetRunner(const RunConfig& config) {
  if (config.dataset == "euroc") {
    return std::make_unique<EuRoCRunner>(config);
  }
  if (config.dataset == "kitti") {
    return std::make_unique<KittiRunner>(config);
  }
  if (config.dataset == "tum") {
    return std::make_unique<TumRunner>(config);
  }
  if (config.dataset == "tumvi") {
    return std::make_unique<TumViRunner>(config);
  }
  throw std::invalid_argument("Unknown dataset: " + config.dataset);
}

// ─────────────────────────────────────────────────────────────────────────────
// CLI Parser
// ─────────────────────────────────────────────────────────────────────────────

namespace {

void validateConfig(const RunConfig& config) {
  // Dataset-sensor validation matrix
  if (config.dataset == "euroc") {
    if (config.sensor != "mono" && config.sensor != "stereo") {
      throw po::error("euroc only supports mono and stereo sensors");
    }
  } else if (config.dataset == "kitti") {
    if (config.sensor != "mono" && config.sensor != "stereo") {
      throw po::error("kitti only supports mono and stereo sensors");
    }
    if (config.inertial) {
      throw po::error("kitti does not support inertial mode");
    }
  } else if (config.dataset == "tum") {
    if (config.sensor != "mono" && config.sensor != "rgbd") {
      throw po::error("tum only supports mono and rgbd sensors");
    }
    if (config.inertial) {
      throw po::error("tum does not support inertial mode");
    }
  } else if (config.dataset == "tumvi") {
    if (config.sensor != "mono" && config.sensor != "stereo") {
      throw po::error("tumvi only supports mono and stereo sensors");
    }
  } else {
    throw po::error("Unknown dataset: " + config.dataset);
  }

  // Sequence requirements
  bool needs_sequences    = (config.dataset == "euroc" || config.dataset == "tumvi");
  bool needs_sequence_dir = (config.dataset == "kitti" || config.dataset == "tum");

  if (needs_sequences && config.sequences.empty()) {
    throw po::error("--sequences is required for dataset " + config.dataset);
  }
  if (needs_sequence_dir && config.sequence_dir.empty()) {
    throw po::error("--sequence-dir is required for dataset " + config.dataset);
  }

  // TUM RGBD needs association file
  if (config.dataset == "tum" && config.sensor == "rgbd" && config.association_file.empty()) {
    throw po::error("--association-file is required for tum+rgbd");
  }

  // Validate output-format if provided
  if (!config.output_format.empty()) {
    if (config.output_format != "euroc" && config.output_format != "tum"
        && config.output_format != "kitti") {
      throw po::error("Invalid output format: " + config.output_format + " (euroc|tum|kitti)");
    }
  }
}

} // namespace

bool parseUnifiedArguments(int argc, char** argv, RunConfig& config) {
  po::options_description desc("orb_slam3_offline options");
  // clang-format off
  desc.add_options()
    ("help,h",           "Show help message")
    ("dataset",          po::value<std::string>(&config.dataset)->required(),
                         "Dataset type: euroc|kitti|tum|tumvi")
    ("sensor",           po::value<std::string>(&config.sensor)->required(),
                         "Sensor type: mono|stereo|rgbd")
    ("inertial",         po::bool_switch(&config.inertial),
                         "Enable inertial mode (euroc, tumvi only)")
    ("vocabulary-file",  po::value<std::string>(&config.vocabulary_file)->required(),
                         "Path to vocabulary text file")
    ("settings-file",    po::value<std::string>(&config.settings_file)->required(),
                         "Path to settings yaml file")
    ("output-dir",       po::value<std::string>(&config.output_dir)->default_value("/tmp"),
                         "Path to output directory")
    ("output-format",    po::value<std::string>(&config.output_format)->default_value(""),
                         "Trajectory output format: euroc|tum|kitti (default: dataset-native)")
    ("no-viewer",        "Disable the Pangolin viewer")
    ("sequences",        po::value<std::vector<std::string>>(&config.sequences)->multitoken(),
                         "Sequence paths (euroc/tumvi)")
    ("sequence-dir",     po::value<std::string>(&config.sequence_dir),
                         "Path to sequence directory (kitti/tum)")
    ("association-file", po::value<std::string>(&config.association_file),
                         "Path to association file (tum+rgbd)");
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

  config.use_viewer = (vm.count("no-viewer") == 0);

  // Validate the configuration
  validateConfig(config);

  // Validate file existence
  if (!fs::is_regular_file(config.vocabulary_file)) {
    throw po::error("Vocabulary path is not a file: " + config.vocabulary_file);
  }
  if (!fs::is_regular_file(config.settings_file)) {
    throw po::error("Settings path is not a file: " + config.settings_file);
  }
  if (!fs::is_directory(config.output_dir)) {
    throw po::error("Output directory does NOT exist: " + config.output_dir);
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Trajectory Saver
// ─────────────────────────────────────────────────────────────────────────────

void saveTrajectory(
  System& slam, TrajectoryFormat format, System::eSensor sensor, const std::string& output_dir
) {
  bool     is_mono = (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);
  fs::path dir(output_dir);

  switch (format) {
    case TrajectoryFormat::kEuRoC:
      slam.SaveTrajectoryEuRoC((dir / "CameraTrajectory.txt").string());
      slam.SaveKeyFrameTrajectoryEuRoC((dir / "KeyFrameTrajectory.txt").string());
      break;
    case TrajectoryFormat::kTUM:
      if (!is_mono) {
        slam.SaveTrajectoryTUM((dir / "CameraTrajectory.txt").string());
      }
      slam.SaveKeyFrameTrajectoryTUM((dir / "KeyFrameTrajectory.txt").string());
      break;
    case TrajectoryFormat::kKITTI:
      if (!is_mono) {
        slam.SaveTrajectoryKITTI((dir / "CameraTrajectory.txt").string());
      } else {
        spdlog::warn("KITTI format does not support monocular. Saving KeyFrame in TUM format.");
        slam.SaveKeyFrameTrajectoryTUM((dir / "KeyFrameTrajectory.txt").string());
      }
      break;
  }
}

} // namespace ORB_SLAM3

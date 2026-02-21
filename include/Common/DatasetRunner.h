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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ImuTypes.h"
#include "System.h"

namespace ORB_SLAM3 {

/// Per-sequence loaded data: images, timestamps, and optional IMU measurements.
struct SequenceData {
  std::vector<std::string> image_filenames;
  std::vector<std::string> image_right_filenames;
  std::vector<std::string> depth_filenames;
  std::vector<double>      timestamps;
  std::vector<double>      imu_timestamps;
  std::vector<cv::Point3f> acc;
  std::vector<cv::Point3f> gyro;
  int                      first_imu = 0;  ///< Next IMU index to consume (advanced by collectImu).
};

/// Trajectory output format for evo evaluation compatibility.
enum class TrajectoryFormat {
  kEuRoC,
  kTUM,
  kKITTI
};

/// Runtime configuration parsed from CLI arguments.
struct RunConfig {
  std::string              vocabulary_file;
  std::string              settings_file;
  std::string              output_dir = "/tmp";
  std::string              output_format;
  bool                     use_viewer = true;
  std::string              dataset;   ///< euroc|kitti|tum|tumvi
  std::string              sensor;    ///< mono|stereo|rgbd
  bool                     inertial = false;
  std::vector<std::string> sequences;
  std::string              sequence_dir;
  std::string              association_file;
};

/// Abstract interface for dataset-specific offline processing.
///
/// Concrete implementations (EuRoCRunner, KittiRunner, TumRunner, TumViRunner)
/// handle dataset-specific loading, frame reading, and IMU collection. Selected
/// once at startup via createDatasetRunner(); virtual dispatch cost is negligible.
class DatasetRunner {
public:
  virtual ~DatasetRunner() = default;

  /// Load all sequence data (images, timestamps, IMU) from disk.
  virtual void load() = 0;

  /// @return Number of sequences loaded.
  [[nodiscard]] virtual int numSequences() const = 0;

  /// @return Number of images in the given sequence.
  [[nodiscard]] virtual int numImages(int seq) const = 0;

  /// @return Camera timestamp for frame @p ni in sequence @p seq.
  [[nodiscard]] double timestamp(int seq, int ni) const {
    return _sequences[seq].timestamps[ni];
  }

  /// Read frame images, applying resize if image_scale != 1.
  /// @param[out] im       Left / mono / RGB image.
  /// @param[out] im_right Right stereo image (empty if N/A).
  /// @param[out] depth    Depth image (empty if N/A).
  /// @return Camera timestamp for this frame.
  [[nodiscard]] virtual double readFrame(
    int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
  ) = 0;

  /// Consume IMU measurements between the previous and current camera frame.
  ///
  /// @note **Mutates internal state**: advances SequenceData::first_imu.
  ///       Must be called exactly once per frame, in order, for correct results.
  /// @return IMU measurements in [t_{ni-1}, t_{ni}]. Empty for ni==0 or non-IMU modes.
  [[nodiscard]] virtual std::vector<IMU::Point> collectImu(int seq, int ni) = 0;

  /// @return ORB-SLAM3 sensor type enum for System construction.
  [[nodiscard]] virtual System::eSensor sensorType() const noexcept = 0;

  /// @return Dataset-native trajectory format for evo evaluation.
  [[nodiscard]] virtual TrajectoryFormat trajectoryFormat() const noexcept = 0;

  /// @return true if CLAHE histogram equalization should be applied (TUM-VI only).
  [[nodiscard]] virtual bool useClahe() const noexcept {
    return false;
  }

  /// @return OpenCV imread flag for this dataset's images.
  [[nodiscard]] virtual int imreadMode() const noexcept {
    return cv::IMREAD_UNCHANGED;
  }

protected:
  std::vector<SequenceData> _sequences;  ///< Loaded per-sequence data.
};

[[nodiscard]] std::unique_ptr<DatasetRunner> createDatasetRunner(const RunConfig& config);

[[nodiscard]] bool parseUnifiedArguments(int argc, char** argv, RunConfig& config);

void saveTrajectory(
  System& slam, TrajectoryFormat format, System::eSensor sensor, const std::string& output_dir
);

} // namespace ORB_SLAM3

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

#include <cassert>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ImuTypes.h"
#include "System.h"

namespace ORB_SLAM3 {

/// @brief Per-sequence loaded data: images, timestamps, and optional IMU measurements.
///
/// Each sequence in a multi-sequence dataset (e.g. EuRoC MH01..MH05) is stored
/// as one SequenceData instance. The @c first_imu cursor is advanced by
/// DatasetRunner::collectImu() during sequential frame processing.
struct SequenceData {
  std::vector<std::string> image_filenames;
  std::vector<std::string> image_right_filenames;
  std::vector<std::string> depth_filenames;
  std::vector<double>      timestamps;
  std::vector<double>      imu_timestamps;
  std::vector<cv::Point3f> acc;
  std::vector<cv::Point3f> gyro;
  int                      first_imu = 0; ///< Next IMU index to consume (advanced by collectImu).
};

/// @brief Trajectory output format for evo evaluation compatibility.
enum class TrajectoryFormat {
  kEuRoC, ///< EuRoC CSV format (timestamp_ns tx ty tz qx qy qz qw).
  kTUM,   ///< TUM format (timestamp tx ty tz qx qy qz qw).
  kKITTI  ///< KITTI format (3x4 row-major transformation matrix).
};

/// @brief Runtime configuration parsed from CLI arguments.
///
/// Populated by parseUnifiedArguments() from command-line flags. Validated
/// against the dataset-sensor compatibility matrix before use.
struct RunConfig {
  std::string              vocabulary_file;
  std::string              settings_file;
  std::string              output_dir = "/tmp";
  std::string              output_format;
  bool                     use_viewer = true;
  std::string              dataset; ///< Dataset type: euroc|kitti|tum|tumvi.
  std::string              sensor;  ///< Sensor mode: mono|stereo|rgbd.
  bool                     inertial = false;
  std::vector<std::string> sequences;
  std::string              sequence_dir;
  std::string              association_file;
};

/// @brief Abstract Strategy interface for dataset-specific offline processing.
///
/// Concrete implementations (EuRoCRunner, KittiRunner, TumRunner, TumViRunner)
/// handle dataset-specific loading, frame reading, and IMU collection. Selected
/// once at startup via createDatasetRunner(); virtual dispatch cost is negligible
/// compared to image I/O and SLAM processing.
///
/// @par Typical usage:
/// @code
///   auto runner = createDatasetRunner(config);
///   runner->load();
///   for (int seq = 0; seq < runner->numSequences(); ++seq)
///     for (int ni = 0; ni < runner->numImages(seq); ++ni) {
///       cv::Mat im, im_right, depth;
///       runner->readFrame(seq, ni, scale, im, im_right, depth);
///       auto imu = runner->collectImu(seq, ni);
///       // ... feed to SLAM system
///     }
/// @endcode
class DatasetRunner {
public:
  virtual ~DatasetRunner() = default;

  /// @brief Load all sequence data (images, timestamps, IMU) from disk.
  /// @throws std::runtime_error If data loading fails (missing files, empty sequences).
  virtual void load() = 0;

  /// @brief Get the number of loaded sequences.
  /// @return Number of sequences (always >= 1 after successful load()).
  [[nodiscard]] virtual int numSequences() const = 0;

  /// @brief Get the number of images in a given sequence.
  /// @param seq Sequence index in [0, numSequences()).
  /// @return Number of images in the sequence.
  [[nodiscard]] virtual int numImages(int seq) const = 0;

  /// @brief Get the camera timestamp for a specific frame.
  /// @param seq Sequence index in [0, numSequences()).
  /// @param ni  Frame index in [0, numImages(seq)).
  /// @return Camera timestamp in seconds.
  [[nodiscard]] double timestamp(int seq, int ni) const {
    assert(seq >= 0 && seq < std::ssize(_sequences));
    assert(ni >= 0 && ni < std::ssize(_sequences[seq].timestamps));
    return _sequences[seq].timestamps[ni];
  }

  /// @brief Read frame images, applying resize if @p image_scale is not 1.0.
  /// @param seq         Sequence index in [0, numSequences()).
  /// @param ni          Frame index in [0, numImages(seq)).
  /// @param image_scale Scale factor from System::GetImageScale().
  /// @param[out] im       Left / mono / RGB image.
  /// @param[out] im_right Right stereo image (empty if N/A).
  /// @param[out] depth    Depth image (empty if N/A).
  /// @return Camera timestamp for this frame.
  /// @throws std::runtime_error If image file cannot be loaded.
  [[nodiscard]] virtual double readFrame(
    int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
  ) = 0;

  /// @brief Consume IMU measurements between the previous and current camera frame.
  ///
  /// Collects all IMU samples with timestamps in (t_{ni-1}, t_{ni}] and returns
  /// them as a vector. Advances the internal SequenceData::first_imu cursor.
  ///
  /// @note **Mutates internal state**: advances SequenceData::first_imu.
  ///       Must be called exactly once per frame, in order, for correct results.
  /// @param seq Sequence index in [0, numSequences()).
  /// @param ni  Frame index in [0, numImages(seq)).
  /// @return IMU measurements in [t_{ni-1}, t_{ni}]. Empty for ni==0 or non-IMU modes.
  [[nodiscard]] virtual std::vector<IMU::Point> collectImu(int seq, int ni) = 0;

  /// @brief Get the ORB-SLAM3 sensor type enum for System construction.
  /// @return Sensor type (MONOCULAR, STEREO, RGBD, or their IMU_ variants).
  [[nodiscard]] virtual System::eSensor sensorType() const noexcept = 0;

  /// @brief Get the dataset-native trajectory format for evo evaluation.
  /// @return Trajectory output format matching the dataset's ground truth convention.
  [[nodiscard]] virtual TrajectoryFormat trajectoryFormat() const noexcept = 0;

  /// @brief Check if CLAHE histogram equalization should be applied.
  /// @return true for TUM-VI (low-contrast images), false otherwise.
  [[nodiscard]] virtual bool useClahe() const noexcept {
    return false;
  }

  /// @brief Get the OpenCV imread flag for this dataset's images.
  /// @return cv::IMREAD_GRAYSCALE for TUM-VI, cv::IMREAD_UNCHANGED for others.
  [[nodiscard]] virtual int imreadMode() const noexcept {
    return cv::IMREAD_UNCHANGED;
  }

  /// @brief Get the sequence string parameter for System constructor.
  ///
  /// TUM-VI passes the output directory so that ORB-SLAM3 can save intermediate
  /// multi-map results. All other datasets return an empty string.
  ///
  /// @return Sequence parameter string (non-empty for TUM-VI only).
  [[nodiscard]] virtual std::string sequenceParam() const noexcept {
    return {};
  }

protected:
  std::vector<SequenceData> _sequences; ///< Loaded per-sequence data.
};

/// @brief Factory: create the appropriate DatasetRunner for the given configuration.
/// @param config Runtime configuration with dataset type and sensor mode.
/// @return Owning pointer to the concrete runner.
/// @throws std::invalid_argument If config.dataset is unknown.
[[nodiscard]] std::unique_ptr<DatasetRunner> createDatasetRunner(const RunConfig& config);

/// @brief Parse unified CLI arguments into a RunConfig.
/// @param argc Argument count from main().
/// @param argv Argument vector from main().
/// @param[out] config Populated configuration on success.
/// @return true if arguments were parsed successfully, false if --help was requested.
/// @throws boost::program_options::error On invalid arguments or failed validation.
[[nodiscard]] bool parseUnifiedArguments(int argc, char** argv, RunConfig& config);

/// @brief Save SLAM trajectory to disk in the specified format.
/// @param slam       Active SLAM system (trajectory is extracted from its map).
/// @param format     Output trajectory format (EuRoC, TUM, or KITTI).
/// @param sensor     Sensor type (determines which trajectories are available).
/// @param output_dir Directory to write trajectory files into.
void saveTrajectory(
  System& slam, TrajectoryFormat format, System::eSensor sensor, const std::string& output_dir
);

} // namespace ORB_SLAM3

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
#include <filesystem>
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
/// as one Sequence instance. The @c first_imu cursor is advanced by
/// DatasetRunner::readIMU() during sequential frame processing.
struct Sequence {
  // Image(s)
  std::vector<double>      img_timestamps;
  std::vector<std::string> img_left_filenames;
  std::vector<std::string> img_right_filenames;
  std::vector<std::string> depth_filenames;
  // IMU (optional)
  std::vector<double>      imu_timestamps;
  std::vector<cv::Point3f> acc;
  std::vector<cv::Point3f> gyro;
  std::size_t              first_imu = 0; ///< Next IMU index to consume (advanced by readIMU).
};

/// @brief Dataset type identifier for factory dispatch and validation.
enum class DatasetType {
  EuRoC,
  TUM,
  TumVI
};

/// @brief Runtime configuration parsed from CLI arguments.
///
/// Populated by parseArgs() from command-line flags. Validated
/// against the dataset-sensor compatibility matrix before use.
struct RunConfig {
  // Principal arguments
  DatasetType           dataset;
  System::eSensor       sensor;
  std::vector<std::filesystem::path> data_dirs;

  // Required files
  std::filesystem::path vocabulary_file;
  std::filesystem::path settings_file;

  // Output
  std::filesystem::path output_dir = "/tmp";
  bool     use_viewer = true;
};

/// @brief Abstract Strategy interface for dataset-specific offline processing.
///
/// Concrete implementations (EuRoCRunner, TumRunner, TumViRunner)
/// handle dataset-specific loading. Frame reading and IMU collection are shared
/// in this base class. Selected once at startup via createDatasetRunner().
///
/// @par Typical usage:
/// @code
///   auto runner = createDatasetRunner(config);
///   runner->load();
///   for (std::size_t seq = 0; seq < runner->numSequences(); ++seq)
///     for (std::size_t frame = 0; frame < runner->numFrames(seq); ++frame) {
///       cv::Mat img_left, img_right, depth;
///       runner->readFrame(seq, frame, scale, img_left, img_right, depth);
///       auto imu = runner->readIMU(seq, frame);
///       // ... feed to SLAM system
///     }
/// @endcode
class DatasetRunner {
public:
  virtual ~DatasetRunner() = default;

  /// @brief Load all sequence data (images, timestamps, IMU) from disk.
  /// @throws std::runtime_error If data loading fails (missing files, empty sequences).
  virtual void load() = 0;

  /// @brief Get the ORB-SLAM3 sensor type enum for System construction.
  /// @return Sensor type (MONOCULAR, STEREO, RGBD, or their IMU_ variants).
  [[nodiscard]] virtual System::eSensor sensor() const noexcept = 0;

  /// @brief Get the number of loaded sequences.
  /// @return Number of sequences (always >= 1 after successful load()).
  [[nodiscard]] std::size_t numSequences() const;

  /// @brief Get the number of images in a given sequence.
  /// @param seq Sequence index in [0, numSequences()).
  /// @return Number of images in the sequence.
  [[nodiscard]] std::size_t numFrames(std::size_t seq) const;

  /// @brief Get the camera timestamp for a specific frame.
  /// @param seq   Sequence index in [0, numSequences()).
  /// @param frame Frame index in [0, numFrames(seq)).
  /// @return Camera timestamp in seconds.
  [[nodiscard]] double timestamp(std::size_t seq, std::size_t frame) const {
    assert(seq < _sequences.size());
    assert(frame < _sequences[seq].img_timestamps.size());
    return _sequences[seq].img_timestamps[frame];
  }

  /// @brief Read frame images, applying resize if @p img_scale is not 1.0.
  /// @param seq       Sequence index in [0, numSequences()).
  /// @param frame     Frame index in [0, numFrames(seq)).
  /// @param img_scale Scale factor from System::GetImageScale().
  /// @param[out] img_left  Left / mono / RGB image.
  /// @param[out] img_right Right stereo image (empty if N/A).
  /// @param[out] depth     Depth image (empty if N/A).
  /// @return Camera timestamp for this frame.
  /// @throws std::runtime_error If image file cannot be loaded.
  [[nodiscard]] double readFrame(
    std::size_t seq,
    std::size_t frame,
    float       img_scale,
    cv::Mat&    img_left,
    cv::Mat&    img_right,
    cv::Mat&    depth
  );

  /// @brief Consume IMU measurements between the previous and current camera frame.
  ///
  /// Collects all IMU samples with timestamps in (t_{frame-1}, t_{frame}] and returns
  /// them as a vector. Advances the internal Sequence::first_imu cursor.
  ///
  /// @note **Mutates internal state**: advances Sequence::first_imu.
  ///       Must be called exactly once per frame, in order, for correct results.
  /// @param seq   Sequence index in [0, numSequences()).
  /// @param frame Frame index in [0, numFrames(seq)).
  /// @return IMU measurements in [t_{frame-1}, t_{frame}]. Empty for frame==0 or non-IMU modes.
  [[nodiscard]] std::vector<IMU::Point> readIMU(std::size_t seq, std::size_t frame);

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
  [[nodiscard]] virtual std::string param() const noexcept {
    return {};
  }

protected:
  std::vector<Sequence> _sequences; ///< Loaded per-sequence data.

  /// @brief Align IMU cursor to the first camera timestamp after loading.
  ///
  /// Sets Sequence::first_imu to the IMU sample just before the first camera frame.
  /// Called by inertial runners (EuRoCRunner, TumViRunner) after loading IMU data.
  void syncImu(Sequence& seq);
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
[[nodiscard]] bool parseArgs(int argc, char** argv, RunConfig& config);

/// @brief Save SLAM trajectory to disk in the format matching the original per-dataset binaries.
///
/// EuRoC / TUM-VI use EuRoC format (nanosecond timestamps, biggest-map selection, IMU body frame).
/// TUM monocular uses KeyFrame-only TUM format (full trajectory guard for mono).
/// TUM non-mono uses full TUM format.
///
/// @param slam       Active SLAM system (trajectory is extracted from its map).
/// @param output_dir Directory to write trajectory files into.
/// @param dataset    Dataset type for format selection.
/// @param sensor     Sensor type for monocular detection.
void saveTrajectories(
  System& slam, const std::filesystem::path& output_dir, DatasetType dataset, System::eSensor sensor
);

} // namespace ORB_SLAM3

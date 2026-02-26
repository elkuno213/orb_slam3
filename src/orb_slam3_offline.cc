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

#include <filesystem>
#include <fmt/core.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/DatasetRunner.h"
#include "LoggingUtils.h"
#include "TicToc.h"

namespace fs = std::filesystem;

/// @brief Dispatch a single frame to the appropriate System::Track*() method.
void trackFrame(
  ORB_SLAM3::System&                        slam,
  ORB_SLAM3::System::eSensor                sensor,
  const double                              timestamp,
  const cv::Mat&                            img_left,
  const cv::Mat&                            img_right,
  const cv::Mat&                            depth,
  const std::vector<ORB_SLAM3::IMU::Point>& imu_meas
) {
  switch (sensor) {
    case ORB_SLAM3::System::MONOCULAR:
    case ORB_SLAM3::System::IMU_MONOCULAR:
      slam.TrackMonocular(img_left, timestamp, imu_meas);
      break;
    case ORB_SLAM3::System::STEREO:
    case ORB_SLAM3::System::IMU_STEREO:
      slam.TrackStereo(img_left, img_right, timestamp, imu_meas);
      break;
    case ORB_SLAM3::System::RGBD:
    case ORB_SLAM3::System::IMU_RGBD:
      slam.TrackRGBD(img_left, depth, timestamp, imu_meas);
      break;
    default:
      throw std::runtime_error(
        "Unsupported sensor type: " + std::to_string(static_cast<int>(sensor))
      );
  }
}

int main(int argc, char** argv) {
  // Load env vars and args.
  spdlog::cfg::load_env_levels();
  spdlog::cfg::load_argv_levels(argc, argv);
  // Initialize application logger.
  ORB_SLAM3::logging::InitializeAppLogger("ORB-SLAM3", false);
  // Add file sink to the application logger.
  const std::string basename  = fs::path(argv[0]).stem().string();
  const std::string logfile   = fmt::format("/tmp/{}.log", basename);
  auto              file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile);
  spdlog::default_logger()->sinks().push_back(file_sink);

  // Parse arguments.
  ORB_SLAM3::RunConfig config;
  try {
    if (!ORB_SLAM3::parseArgs(argc, argv, config)) {
      return 0; // --help requested; parseArgs() already printed usage
    }
  } catch (const std::exception& e) {
    spdlog::error("Error when parsing arguments: {}", e.what());
    return 1;
  }

  // Run.
  try {
    // Create and load dataset runner.
    auto runner = ORB_SLAM3::createDatasetRunner(config);
    runner->load();

    const std::size_t num_sequences = runner->numSequences();
    spdlog::info("Number of sequences: {}", num_sequences);

    // Create SLAM system. The sequence parameter is dataset-specific (non-empty for TUM-VI).
    ORB_SLAM3::System slam(
      config.vocabulary_file.string(),
      config.settings_file.string(),
      runner->sensor(),
      config.use_viewer,
      0,
      runner->param()
    );
    const float image_scale = slam.GetImageScale();

    // CLAHE setup for TUM-VI.
    cv::Ptr<cv::CLAHE> clahe;
    if (runner->useClahe()) {
      clahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // TUM-VI defaults from ORB-SLAM3 paper
    }

    // Process all sequences.
    for (std::size_t seq = 0; seq < num_sequences; seq++) {
      const std::size_t num_frames = runner->numFrames(seq);
      spdlog::info("Processing sequence {} with {} images...", seq, num_frames);

      ORB_SLAM3::TicToc timer;

      for (std::size_t frame = 0; frame < num_frames; frame++) {
        cv::Mat img_left, img_right, depth;

        // Read frame.
        const double timestamp
          = runner->readFrame(seq, frame, image_scale, img_left, img_right, depth);

        // Apply CLAHE if needed.
        if (clahe) {
          clahe->apply(img_left, img_left);
          if (!img_right.empty()) {
            clahe->apply(img_right, img_right);
          }
        }

        // Collect IMU measurements and track.
        const auto imu_meas = runner->readIMU(seq, frame);

        timer.tic();
        trackFrame(slam, runner->sensor(), timestamp, img_left, img_right, depth, imu_meas);

        // Real-time pacing: sleep to match camera frame rate.
        double frame_interval = 0.0;
        if (frame + 1 < num_frames) {
          frame_interval = runner->timestamp(seq, frame + 1) - timestamp;
        } else if (frame > 0) {
          frame_interval = timestamp - runner->timestamp(seq, frame - 1);
        }
        timer.paceSinceTic(frame_interval);
      }

      if (seq < num_sequences - 1) {
        // Save per-sequence SubMap trajectories
        const fs::path submap_dir = config.output_dir / "submap";
        fs::create_directories(submap_dir);
        ORB_SLAM3::saveTrajectories(slam, submap_dir, config.dataset, runner->sensor());
        spdlog::info("SubMap {} saved to {}", seq, submap_dir.string());

        spdlog::info("Changing the dataset...");
        slam.ChangeDataset();
      }
    }

    // Stop all threads.
    slam.Shutdown();

    // Save trajectory.
    ORB_SLAM3::saveTrajectories(slam, config.output_dir, config.dataset, runner->sensor());
    spdlog::info("Trajectory saved to {}", config.output_dir.string());
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
    return 1;
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
    return 1;
  }

  return 0;
}

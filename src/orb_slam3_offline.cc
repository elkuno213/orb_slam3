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

#include <chrono>
#include <filesystem>
#include <thread>
#include <fmt/core.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/DatasetRunner.h"
#include "LoggingUtils.h"

namespace fs = std::filesystem;

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
    if (!ORB_SLAM3::parseUnifiedArguments(argc, argv, config)) {
      return 0;
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

    const int num_seq = runner->numSequences();
    spdlog::info("Number of sequences: {}", num_seq);

    // Create SLAM system. The sequence parameter is dataset-specific (non-empty for TUM-VI).
    ORB_SLAM3::System slam(
      config.vocabulary_file,
      config.settings_file,
      runner->sensorType(),
      config.use_viewer,
      0,
      runner->sequenceParam()
    );
    const float image_scale = slam.GetImageScale();

    // CLAHE setup for TUM-VI.
    cv::Ptr<cv::CLAHE> clahe;
    if (runner->useClahe()) {
      clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    // Determine trajectory format.
    ORB_SLAM3::TrajectoryFormat traj_format = runner->trajectoryFormat();
    if (!config.output_format.empty()) {
      if (config.output_format == "euroc") {
        traj_format = ORB_SLAM3::TrajectoryFormat::kEuRoC;
      } else if (config.output_format == "tum") {
        traj_format = ORB_SLAM3::TrajectoryFormat::kTUM;
      } else if (config.output_format == "kitti") {
        traj_format = ORB_SLAM3::TrajectoryFormat::kKITTI;
      }
    }

    // Process all sequences.
    for (int seq = 0; seq < num_seq; seq++) {
      const int n_images = runner->numImages(seq);
      spdlog::info("Processing sequence {} with {} images...", seq, n_images);

      for (int ni = 0; ni < n_images; ni++) {
        cv::Mat im, im_right, depth;

        // Read frame.
        const double tframe = runner->readFrame(seq, ni, image_scale, im, im_right, depth);

        // Apply CLAHE if needed.
        if (clahe) {
          clahe->apply(im, im);
          if (!im_right.empty()) {
            clahe->apply(im_right, im_right);
          }
        }

        // Collect IMU measurements.
        const auto imu_meas = runner->collectImu(seq, ni);

        // Track.
        const auto t1 = std::chrono::steady_clock::now();

        switch (runner->sensorType()) {
          case ORB_SLAM3::System::MONOCULAR:
          case ORB_SLAM3::System::IMU_MONOCULAR:
            slam.TrackMonocular(im, tframe, imu_meas);
            break;
          case ORB_SLAM3::System::STEREO:
          case ORB_SLAM3::System::IMU_STEREO:
            slam.TrackStereo(im, im_right, tframe, imu_meas);
            break;
          case ORB_SLAM3::System::RGBD:
          case ORB_SLAM3::System::IMU_RGBD:
            slam.TrackRGBD(im, depth, tframe, imu_meas);
            break;
          default:
            spdlog::error("Unsupported sensor type: {}", static_cast<int>(runner->sensorType()));
            return 1;
        }

        const auto   t2 = std::chrono::steady_clock::now();
        const double ttrack
          = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        // Real-time pacing: wait if tracking was faster than the frame interval.
        double frame_interval = 0.0;
        if (ni < n_images - 1) {
          frame_interval = runner->timestamp(seq, ni + 1) - tframe;
        } else if (ni > 0) {
          frame_interval = tframe - runner->timestamp(seq, ni - 1);
        }
        if (ttrack < frame_interval) {
          std::this_thread::sleep_for(std::chrono::duration<double>(frame_interval - ttrack));
        }
      }

      if (seq < num_seq - 1) {
        // Save per-sequence SubMap trajectories (feature parity with legacy binaries).
        const fs::path submap_dir = fs::path(config.output_dir) / "SubMaps";
        fs::create_directories(submap_dir);
        ORB_SLAM3::saveTrajectory(slam, traj_format, runner->sensorType(), submap_dir.string());
        spdlog::info("SubMap {} saved to {}", seq, submap_dir.string());

        spdlog::info("Changing the dataset...");
        slam.ChangeDataset();
      }
    }

    // Stop all threads.
    slam.Shutdown();

    // Save trajectory.
    ORB_SLAM3::saveTrajectory(slam, traj_format, runner->sensorType(), config.output_dir);
    spdlog::info("Trajectory saved to {}", config.output_dir);
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
    return 1;
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
    return 1;
  }

  return 0;
}

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

/**
 * Unified RealSense binary replacing 10 individual RealSense example binaries.
 *
 * Supported modes (--device d435i):
 *   --sensor mono                    → Monocular (IR left)
 *   --sensor mono   --inertial       → Monocular-Inertial (IR left + IMU)
 *   --sensor stereo                  → Stereo (IR left + right)
 *   --sensor stereo --inertial       → Stereo-Inertial (IR left + right + IMU)
 *   --sensor rgbd                    → RGB-D (color + depth, aligned)
 *   --sensor rgbd   --inertial       → RGB-D-Inertial (color + depth + IMU, aligned)
 *
 * Supported modes (--device t265):
 *   --sensor mono                    → Monocular (fisheye 1)
 *   --sensor mono   --inertial       → Monocular-Inertial (fisheye 1 + IMU)
 *   --sensor stereo                  → Stereo (fisheye 1 + 2)
 *   --sensor stereo --inertial       → Stereo-Inertial (fisheye 1 + 2 + IMU)
 */

#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <sstream>
#include <boost/program_options.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/RealSense.h"
#include "ImuTypes.h"
#include "LoggingUtils.h"
#include "System.h"

namespace fs = std::filesystem;
namespace po = boost::program_options;

// ──────────────────────────────────────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────────────────────────────────────

struct RealsenseConfig {
  std::string vocabulary_file;
  std::string settings_file;
  std::string output_dir = "/tmp";
  bool        use_viewer = true;
  std::string sensor;  // mono, stereo, rgbd
  std::string device;  // d435i, t265
  bool        inertial = false;
};

ORB_SLAM3::System::eSensor determineSensorType(const RealsenseConfig& config) {
  if (config.sensor == "mono") {
    return config.inertial ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR;
  }
  if (config.sensor == "stereo") {
    return config.inertial ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO;
  }
  if (config.sensor == "rgbd") {
    return config.inertial ? ORB_SLAM3::System::IMU_RGBD : ORB_SLAM3::System::RGBD;
  }
  throw std::runtime_error("Unknown sensor type: " + config.sensor);
}

bool parseArguments(int argc, char** argv, RealsenseConfig& config) {
  po::options_description desc("Unified RealSense binary for ORB-SLAM3");
  // clang-format off
  desc.add_options()
    ("help,h", "Show help message")
    ("vocabulary-file", po::value<std::string>(&config.vocabulary_file)->required(),
      "Path to vocabulary text file")
    ("settings-file", po::value<std::string>(&config.settings_file)->required(),
      "Path to settings YAML file")
    ("output-dir", po::value<std::string>(&config.output_dir)->default_value("/tmp"),
      "Path to output directory")
    ("no-viewer", "Disable the Pangolin viewer")
    ("sensor", po::value<std::string>(&config.sensor)->required(),
      "Sensor mode: mono, stereo, rgbd")
    ("device", po::value<std::string>(&config.device)->required(),
      "RealSense device: d435i, t265")
    ("inertial", "Enable inertial (IMU) mode");
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

  config.use_viewer = vm.count("no-viewer") == 0;
  config.inertial   = vm.count("inertial") > 0;

  // Validate sensor.
  if (config.sensor != "mono" && config.sensor != "stereo" && config.sensor != "rgbd") {
    throw po::error("Invalid --sensor value: " + config.sensor + " (must be mono, stereo, or rgbd)");
  }

  // Validate device.
  if (config.device != "d435i" && config.device != "t265") {
    throw po::error("Invalid --device value: " + config.device + " (must be d435i or t265)");
  }

  // Validate combinations.
  if (config.device == "t265" && config.sensor == "rgbd") {
    throw po::error("T265 does not support RGB-D mode");
  }

  // Validate file paths.
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

// ──────────────────────────────────────────────────────────────────────────────
// Signal handling
// ──────────────────────────────────────────────────────────────────────────────

volatile sig_atomic_t g_continue_session = 1;

void exitLoopHandler(int /*s*/) {
  spdlog::info("Finishing session");
  g_continue_session = 0;
}

void installSignalHandler() {
  struct sigaction sa {};
  sa.sa_handler = exitLoopHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
}

// ──────────────────────────────────────────────────────────────────────────────
// D435i device setup
// ──────────────────────────────────────────────────────────────────────────────

void configureD435iDevice(rs2::context& ctx, const RealsenseConfig& config) {
  rs2::device_list devices = ctx.query_devices();
  if (devices.size() == 0) {
    throw std::runtime_error("No RealSense device connected");
  }

  rs2::device                 selected_device = devices[0];
  std::vector<rs2::sensor> sensors         = selected_device.query_sensors();
  int                         sensor_index    = 0;

  for (rs2::sensor sensor : sensors) {
    if (!sensor.supports(RS2_CAMERA_INFO_NAME)) continue;
    ++sensor_index;

    if (sensor_index == 1) {
      // Stereo/IR module
      sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
      if (config.sensor == "rgbd") {
        // Emitter on for depth information
        sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 50000);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
      } else {
        // Emitter off for IR stereo
        sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 5000);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
      }
    }

    ORB_SLAM3::RealSense::get_sensor_option(sensor);

    if (sensor_index == 2) {
      // RGB camera
      if (config.sensor == "rgbd") {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
      } else {
        sensor.set_option(RS2_OPTION_EXPOSURE, 100.f);
      }
    }

    if (sensor_index == 3 && config.inertial) {
      sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
    }
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// Pipeline configuration
// ──────────────────────────────────────────────────────────────────────────────

void configurePipeline(rs2::config& cfg, const RealsenseConfig& config) {
  if (config.device == "d435i") {
    if (config.sensor == "rgbd") {
      // RGB + depth streams
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
      cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    } else if (config.sensor == "mono") {
      // Single IR stream
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    } else {
      // Stereo IR streams
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
      cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    }
  } else {
    // T265: fisheye streams (must enable both even for mono)
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8, 30);
  }

  // IMU streams
  if (config.inertial) {
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// Intrinsics logging
// ──────────────────────────────────────────────────────────────────────────────

void logIntrinsics(const std::string& label, const rs2_intrinsics& intr) {
  spdlog::info(
    R"(
    {} camera parameters:
      Intrinsics:
        fx: {:.6f}
        fy: {:.6f}
        cx: {:.6f}
        cy: {:.6f}
      Resolution: {}x{}
      Distortion coefficients: [{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]
      Model: {}
  )",
    label,
    intr.fx,
    intr.fy,
    intr.ppx,
    intr.ppy,
    intr.width,
    intr.height,
    intr.coeffs[0],
    intr.coeffs[1],
    intr.coeffs[2],
    intr.coeffs[3],
    intr.coeffs[4],
    intr.model
  );
}

// ──────────────────────────────────────────────────────────────────────────────
// T265 non-inertial modes (synchronous frame polling)
// ──────────────────────────────────────────────────────────────────────────────

int runT265Sync(const RealsenseConfig& config) {
  rs2::pipeline pipe;
  rs2::config   cfg;
  configurePipeline(cfg, config);

  rs2::pipeline_profile pipe_profile = pipe.start(cfg);

  auto sensor_type = determineSensorType(config);
  ORB_SLAM3::System slam(
    config.vocabulary_file, config.settings_file, sensor_type, config.use_viewer, 0, config.output_dir
  );
  float image_scale = slam.GetImageScale();

  rs2::stream_profile fisheye_left = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
  rs2_intrinsics      intr_left    = fisheye_left.as<rs2::video_stream_profile>().get_intrinsics();
  int                 width_img    = intr_left.width;
  int                 height_img   = intr_left.height;

  bool use_stereo = (config.sensor == "stereo");

  while (g_continue_session) {
    rs2::frameset frame_set = pipe.wait_for_frames();
    double        timestamp = frame_set.get_timestamp();

    if (rs2::video_frame image_frame = frame_set.first_or_default(RS2_STREAM_FISHEYE)) {
      rs2::video_frame frame_left = frame_set.get_fisheye_frame(1);
      cv::Mat          im_left(
        cv::Size(width_img, height_img), CV_8UC1, (void*)(frame_left.get_data()), cv::Mat::AUTO_STEP
      );

      if (image_scale != 1.f) {
        int width  = im_left.cols * image_scale;
        int height = im_left.rows * image_scale;
        cv::resize(im_left, im_left, cv::Size(width, height));
      }

      if (use_stereo) {
        rs2::video_frame frame_right = frame_set.get_fisheye_frame(2);
        cv::Mat          im_right(
          cv::Size(width_img, height_img),
          CV_8UC1,
          (void*)(frame_right.get_data()),
          cv::Mat::AUTO_STEP
        );
        if (image_scale != 1.f) {
          int width  = im_right.cols * image_scale;
          int height = im_right.rows * image_scale;
          cv::resize(im_right, im_right, cv::Size(width, height));
        }
        slam.TrackStereo(im_left, im_right, timestamp);
      } else {
        slam.TrackMonocular(im_left, timestamp);
      }
    }
  }

  pipe.stop();
  slam.Shutdown();
  return 0;
}

// ──────────────────────────────────────────────────────────────────────────────
// Callback-based inertial mode (D435i and T265 with IMU)
// ──────────────────────────────────────────────────────────────────────────────

int runWithImuCallback(const RealsenseConfig& config) {
  // Device setup (D435i only; T265 needs no explicit device config).
  rs2::context ctx;
  if (config.device == "d435i") {
    configureD435iDevice(ctx, config);
  }

  rs2::pipeline pipe;
  rs2::config   cfg;
  configurePipeline(cfg, config);

  bool use_rgbd  = (config.sensor == "rgbd");
  bool use_imu   = config.inertial;
  bool use_left  = true;
  bool use_right = (config.sensor == "stereo");

  // RGBD depth alignment setup
  rs2::pipeline_profile initial_profile;
  rs2_stream            align_to = RS2_STREAM_ANY;
  rs2::align            align(RS2_STREAM_COLOR);
  rs2::frameset         fs_slam;

  if (use_rgbd) {
    // Start-stop to get the initial profile for alignment
    initial_profile = pipe.start(cfg);
    pipe.stop();
    align_to = ORB_SLAM3::RealSense::find_stream_to_align(initial_profile.get_streams());
    align    = rs2::align(align_to);
  }

  // IMU state
  double     offset                 = 0; // ms
  std::mutex imu_mutex;
  std::condition_variable cond_image_rec;

  std::vector<double>     v_gyro_timestamp;
  std::vector<rs2_vector> v_gyro_data;
  double                  prev_accel_timestamp    = 0;
  rs2_vector              prev_accel_data         = {};
  double                  current_accel_timestamp = 0;
  rs2_vector              current_accel_data      = {};
  std::vector<double>     v_accel_timestamp_sync;
  std::vector<rs2_vector> v_accel_data_sync;

  // Image state
  cv::Mat im_cv, im_right_cv;
  int     width_img       = 0;
  int     height_img      = 0;
  double  timestamp_image = -1.0;
  bool    image_ready     = false;
  int     count_im_buffer = 0;

  rs2::pipeline_profile pipe_profile;

  // IMU callback
  auto imu_callback = [&](const rs2::frame& frame) {
    std::unique_lock<std::mutex> lock(imu_mutex);

    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      count_im_buffer++;

      double new_timestamp = fs.get_timestamp() * 1e-3;
      if (std::abs(timestamp_image - new_timestamp) < 0.001) {
        count_im_buffer--;
        return;
      }

      if (use_rgbd) {
        // Check for profile changes (RGBD alignment)
        if (ORB_SLAM3::RealSense::profile_changed(
              pipe.get_active_profile().get_streams(), pipe_profile.get_streams()
            )) {
          pipe_profile = pipe.get_active_profile();
          align_to     = ORB_SLAM3::RealSense::find_stream_to_align(pipe_profile.get_streams());
          align        = rs2::align(align_to);
        }
        // Store frameset for later alignment (outside callback for performance)
        fs_slam = fs;
      } else if (config.device == "d435i") {
        // D435i IR frames
        if (use_right) {
          rs2::video_frame ir_left  = fs.get_infrared_frame(1);
          rs2::video_frame ir_right = fs.get_infrared_frame(2);
          im_cv = cv::Mat(
            cv::Size(width_img, height_img), CV_8U, (void*)(ir_left.get_data()), cv::Mat::AUTO_STEP
          );
          im_right_cv = cv::Mat(
            cv::Size(width_img, height_img), CV_8U, (void*)(ir_right.get_data()), cv::Mat::AUTO_STEP
          );
        } else {
          rs2::video_frame ir_left = fs.get_infrared_frame(1);
          im_cv = cv::Mat(
            cv::Size(width_img, height_img), CV_8U, (void*)(ir_left.get_data()), cv::Mat::AUTO_STEP
          );
        }
      } else {
        // T265 fisheye frames
        if (use_right) {
          rs2::video_frame fe_left  = fs.get_fisheye_frame(1);
          rs2::video_frame fe_right = fs.get_fisheye_frame(2);
          im_cv = cv::Mat(
            cv::Size(width_img, height_img), CV_8U, (void*)(fe_left.get_data()), cv::Mat::AUTO_STEP
          );
          im_right_cv = cv::Mat(
            cv::Size(width_img, height_img), CV_8U, (void*)(fe_right.get_data()), cv::Mat::AUTO_STEP
          );
        } else {
          rs2::video_frame fe_left = fs.get_fisheye_frame(1);
          im_cv = cv::Mat(
            cv::Size(width_img, height_img), CV_8U, (void*)(fe_left.get_data()), cv::Mat::AUTO_STEP
          );
        }
      }

      timestamp_image = new_timestamp;
      image_ready     = true;

      // Sync pending accel measurements to gyro timestamps
      if (use_imu) {
        while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
          int    idx         = v_accel_timestamp_sync.size();
          double target_time = v_gyro_timestamp[idx];

          rs2_vector interp_data = ORB_SLAM3::RealSense::interpolateMeasure(
            target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp
          );
          v_accel_data_sync.push_back(interp_data);
          v_accel_timestamp_sync.push_back(target_time);
        }
      }

      lock.unlock();
      cond_image_rec.notify_all();
    } else if (use_imu) {
      if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
        if (m_frame.get_profile().stream_name() == "Gyro") {
          v_gyro_data.push_back(m_frame.get_motion_data());
          v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
        } else if (m_frame.get_profile().stream_name() == "Accel") {
          prev_accel_timestamp = current_accel_timestamp;
          prev_accel_data      = current_accel_data;
          current_accel_data      = m_frame.get_motion_data();
          current_accel_timestamp = (m_frame.get_timestamp() + offset) * 1e-3;

          while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
            int    idx         = v_accel_timestamp_sync.size();
            double target_time = v_gyro_timestamp[idx];
            rs2_vector interp_data = ORB_SLAM3::RealSense::interpolateMeasure(
              target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp
            );
            v_accel_data_sync.push_back(interp_data);
            v_accel_timestamp_sync.push_back(target_time);
          }
        }
      }
    }
  };

  // Start pipeline with callback
  pipe_profile = pipe.start(cfg, imu_callback);

  // Get intrinsics
  rs2::stream_profile cam_stream;
  if (config.device == "d435i") {
    if (use_rgbd) {
      cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    } else {
      cam_stream = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
    }
  } else {
    cam_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
  }

  rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
  width_img                     = intrinsics_cam.width;
  height_img                    = intrinsics_cam.height;
  logIntrinsics("Left", intrinsics_cam);

  // Log right camera intrinsics for stereo D435i
  if (use_right && config.device == "d435i") {
    rs2::stream_profile cam_right_stream = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);
    rs2_intrinsics      intr_right = cam_right_stream.as<rs2::video_stream_profile>().get_intrinsics();
    logIntrinsics("Right", intr_right);
  }

  // Create SLAM system
  auto sensor_type = determineSensorType(config);
  ORB_SLAM3::System slam(
    config.vocabulary_file, config.settings_file, sensor_type, config.use_viewer, 0, config.output_dir
  );
  float image_scale = slam.GetImageScale();

  std::vector<ORB_SLAM3::IMU::Point> imu_meas;

  // Clear IMU vectors
  v_gyro_data.clear();
  v_gyro_timestamp.clear();
  v_accel_data_sync.clear();
  v_accel_timestamp_sync.clear();

  // Main processing loop
  while (!slam.isShutDown()) {
    std::vector<rs2_vector> v_gyro;
    std::vector<double>     v_gyro_times;
    std::vector<rs2_vector> v_accel;
    std::vector<double>     v_accel_times;
    rs2::frameset           local_fs;

    {
      std::unique_lock<std::mutex> lk(imu_mutex);
      if (!image_ready) {
        cond_image_rec.wait(lk);
      }

      if (count_im_buffer > 1) {
        spdlog::warn("Dropped frames: {}", count_im_buffer - 1);
      }
      count_im_buffer = 0;

      // Final accel sync
      if (use_imu) {
        while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
          int    idx         = v_accel_timestamp_sync.size();
          double target_time = v_gyro_timestamp[idx];
          rs2_vector interp_data = ORB_SLAM3::RealSense::interpolateMeasure(
            target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp
          );
          v_accel_data_sync.push_back(interp_data);
          v_accel_timestamp_sync.push_back(target_time);
        }
      }

      // Copy data from callback
      if (use_imu) {
        v_gyro       = v_gyro_data;
        v_gyro_times = v_gyro_timestamp;
        v_accel      = v_accel_data_sync;
        v_accel_times = v_accel_timestamp_sync;
      }

      double timestamp = timestamp_image;

      if (use_rgbd) {
        local_fs = fs_slam;
      }

      cv::Mat im, im_right, depth;

      if (!use_rgbd) {
        im = im_cv.clone();
        if (use_right) {
          im_right = im_right_cv.clone();
        }
      }

      // Clear IMU vectors
      if (use_imu) {
        v_gyro_data.clear();
        v_gyro_timestamp.clear();
        v_accel_data_sync.clear();
        v_accel_timestamp_sync.clear();
      }

      image_ready = false;
      lk.unlock();

      // RGBD: perform depth alignment outside the lock
      if (use_rgbd) {
        auto processed = align.process(local_fs);
        rs2::video_frame color_frame = processed.first(align_to);
        rs2::depth_frame depth_frame = processed.get_depth_frame();
        im = cv::Mat(
          cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP
        );
        depth = cv::Mat(
          cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP
        );
      }

      // Build IMU measurement vector
      if (use_imu) {
        for (size_t i = 0; i < v_gyro.size(); ++i) {
          ORB_SLAM3::IMU::Point point(
            v_accel[i].x, v_accel[i].y, v_accel[i].z,
            v_gyro[i].x, v_gyro[i].y, v_gyro[i].z,
            v_gyro_times[i]
          );
          imu_meas.push_back(point);
        }
      }

      // Image scaling
      if (image_scale != 1.f) {
        int w = im.cols * image_scale;
        int h = im.rows * image_scale;
        cv::resize(im, im, cv::Size(w, h));
        if (use_right && !im_right.empty()) {
          cv::resize(im_right, im_right, cv::Size(w, h));
        }
        if (use_rgbd && !depth.empty()) {
          cv::resize(depth, depth, cv::Size(w, h));
        }
      }

      // Track
      switch (sensor_type) {
        case ORB_SLAM3::System::MONOCULAR:
          slam.TrackMonocular(im, timestamp);
          break;
        case ORB_SLAM3::System::IMU_MONOCULAR:
          slam.TrackMonocular(im, timestamp, imu_meas);
          break;
        case ORB_SLAM3::System::STEREO:
          slam.TrackStereo(im, im_right, timestamp);
          break;
        case ORB_SLAM3::System::IMU_STEREO:
          slam.TrackStereo(im, im_right, timestamp, imu_meas);
          break;
        case ORB_SLAM3::System::RGBD:
          slam.TrackRGBD(im, depth, timestamp);
          break;
        case ORB_SLAM3::System::IMU_RGBD:
          slam.TrackRGBD(im, depth, timestamp, imu_meas);
          break;
        default:
          spdlog::error("Unsupported sensor type: {}", static_cast<int>(sensor_type));
          return 1;
      }

      imu_meas.clear();
    }
  }

  return 0;
}

// ──────────────────────────────────────────────────────────────────────────────
// D435i non-inertial modes (callback for frame sync, no IMU)
// ──────────────────────────────────────────────────────────────────────────────

int runD435iSync(const RealsenseConfig& config) {
  // The D435i non-inertial modes still use a callback for frame synchronization
  // but without IMU data. We reuse the callback-based path.
  return runWithImuCallback(config);
}

// ──────────────────────────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────────────────────────

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
  RealsenseConfig config;
  try {
    if (!parseArguments(argc, argv, config)) {
      return 0;
    }
  } catch (const std::exception& e) {
    spdlog::error("Error when parsing arguments: {}", e.what());
    return 1;
  }

  spdlog::info(
    "Starting ORB-SLAM3 RealSense: device={}, sensor={}, inertial={}",
    config.device,
    config.sensor,
    config.inertial
  );

  // Run.
  try {
    installSignalHandler();

    // T265 non-inertial modes use synchronous polling
    if (config.device == "t265" && !config.inertial) {
      return runT265Sync(config);
    }

    // All D435i modes and T265 inertial modes use callback
    if (config.device == "d435i") {
      rs2::context ctx;
      configureD435iDevice(ctx, config);
    }
    return runWithImuCallback(config);
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
    return 1;
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
    return 1;
  }
}

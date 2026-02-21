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

#include "Common/DatasetRunner.h"

namespace ORB_SLAM3 {

class EuRoCRunner : public DatasetRunner {
public:
  explicit EuRoCRunner(const RunConfig& config);

  void              load() override;
  [[nodiscard]] int numSequences() const override;
  [[nodiscard]] int numImages(int seq) const override;
  [[nodiscard]] double readFrame(
    int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
  ) override;
  [[nodiscard]] std::vector<IMU::Point> collectImu(int seq, int ni) override;
  [[nodiscard]] System::eSensor         sensorType() const noexcept override;
  [[nodiscard]] TrajectoryFormat        trajectoryFormat() const noexcept override;

private:
  System::eSensor          _sensor_type;
  bool                     _has_imu;
  std::vector<std::string> _raw_sequences;
};

class KittiRunner : public DatasetRunner {
public:
  explicit KittiRunner(const RunConfig& config);

  void              load() override;
  [[nodiscard]] int numSequences() const override;
  [[nodiscard]] int numImages(int seq) const override;
  [[nodiscard]] double readFrame(
    int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
  ) override;
  [[nodiscard]] std::vector<IMU::Point> collectImu(int seq, int ni) override;
  [[nodiscard]] System::eSensor         sensorType() const noexcept override;
  [[nodiscard]] TrajectoryFormat        trajectoryFormat() const noexcept override;

private:
  System::eSensor _sensor_type;
  std::string     _sequence_dir;
};

class TumRunner : public DatasetRunner {
public:
  explicit TumRunner(const RunConfig& config);

  void              load() override;
  [[nodiscard]] int numSequences() const override;
  [[nodiscard]] int numImages(int seq) const override;
  [[nodiscard]] double readFrame(
    int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
  ) override;
  [[nodiscard]] std::vector<IMU::Point> collectImu(int seq, int ni) override;
  [[nodiscard]] System::eSensor         sensorType() const noexcept override;
  [[nodiscard]] TrajectoryFormat        trajectoryFormat() const noexcept override;

private:
  System::eSensor _sensor_type;
  std::string     _sequence_dir;
  std::string     _association_file;
};

class TumViRunner : public DatasetRunner {
public:
  explicit TumViRunner(const RunConfig& config);

  void              load() override;
  [[nodiscard]] int numSequences() const override;
  [[nodiscard]] int numImages(int seq) const override;
  [[nodiscard]] double readFrame(
    int seq, int ni, float image_scale, cv::Mat& im, cv::Mat& im_right, cv::Mat& depth
  ) override;
  [[nodiscard]] std::vector<IMU::Point> collectImu(int seq, int ni) override;
  [[nodiscard]] System::eSensor         sensorType() const noexcept override;
  [[nodiscard]] TrajectoryFormat        trajectoryFormat() const noexcept override;
  [[nodiscard]] bool useClahe() const noexcept override;
  [[nodiscard]] int  imreadMode() const noexcept override;

private:
  System::eSensor          _sensor_type;
  bool                     _has_imu;
  bool                     _is_stereo;
  std::vector<std::string> _raw_sequences;
};

} // namespace ORB_SLAM3

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

#include <filesystem>
#include <vector>
#include "Common/DatasetRunner.h"

namespace ORB_SLAM3 {

/// @brief DatasetRunner for EuRoC MAV datasets (mav0/cam0, mav0/imu0 layout).
///
/// Supports mono, stereo, mono-inertial, and stereo-inertial sensor modes.
/// Loads left (cam0) and optional right (cam1) images from CSV-indexed paths,
/// and optional IMU data from imu0/data.csv.
class EuRoCRunner : public DatasetRunner {
public:
  /// @param config Must have dataset == DatasetType::EuRoC.
  explicit EuRoCRunner(const RunConfig& config);

  void                          load() override;
  [[nodiscard]] System::eSensor sensor() const noexcept override;

private:
  System::eSensor                    _sensor;
  bool                               _inertial;
  std::vector<std::filesystem::path> _data_dirs;
};

} // namespace ORB_SLAM3

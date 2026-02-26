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
#include <string>
#include <vector>
#include "Common/DatasetRunner.h"

namespace ORB_SLAM3 {

/// @brief DatasetRunner for TUM-VI datasets (mav0/cam0, mav0/imu0 layout).
///
/// Supports mono, stereo, and mono-inertial sensor modes. Images are read as
/// grayscale with CLAHE histogram equalization for low-contrast TUM-VI sequences.
/// Passes the output directory as the sequence parameter for multi-map saving.
class TumViRunner : public DatasetRunner {
public:
  /// @param config Must have dataset == DatasetType::TumVI.
  explicit TumViRunner(const RunConfig& config);

  void                          load() override;
  [[nodiscard]] System::eSensor sensor() const noexcept override;
  [[nodiscard]] bool            useClahe() const noexcept override;   ///< Always true.
  [[nodiscard]] int             imreadMode() const noexcept override; ///< cv::IMREAD_GRAYSCALE.
  [[nodiscard]] std::string     param() const noexcept override;      ///< Returns output directory.

private:
  System::eSensor                    _sensor;
  bool                               _inertial;
  bool                               _is_stereo;
  std::vector<std::filesystem::path> _data_dirs;
  std::filesystem::path              _output_dir;
};

} // namespace ORB_SLAM3

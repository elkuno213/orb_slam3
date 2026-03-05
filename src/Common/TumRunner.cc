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

#include "Common/TumRunner.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace {

/// @brief Entry from a TUM-format text file: (timestamp, data_string).
using TumEntry = std::pair<double, std::string>;

/// @brief Parse a TUM-format text file (3 header lines, then "timestamp data" per line).
/// Skips empty lines and lines starting with '#'.
std::vector<TumEntry> readTumFile(const fs::path& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open TUM file: " + path.string());
  }

  // Skip 3 header lines.
  std::string line;
  for (int i = 0; i < 3 && std::getline(file, line); ++i) {
  }

  std::vector<TumEntry> entries;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    double             timestamp;
    std::string        data;
    if (ss >> timestamp >> data) {
      entries.emplace_back(timestamp, std::move(data));
    }
  }
  return entries;
}

/// @brief Associate RGB and depth timestamps using sorted binary search.
///
/// Sorts both lists by timestamp, then for each RGB entry uses std::lower_bound
/// to find the nearest depth entry. O(n log m) instead of O(n*m).
void associateRGBDepth(
  std::vector<TumEntry>     rgb_entries,
  std::vector<TumEntry>     depth_entries,
  std::vector<double>&      timestamps,
  std::vector<std::string>& rgb_filenames,
  std::vector<std::string>& depth_filenames,
  double                    max_difference = 0.02 // TODO(VuHoi): extract to named constant.
) {
  auto by_timestamp = [](const TumEntry& a, const TumEntry& b) {
    return a.first < b.first;
  };
  std::sort(rgb_entries.begin(), rgb_entries.end(), by_timestamp);
  std::sort(depth_entries.begin(), depth_entries.end(), by_timestamp);

  timestamps.reserve(rgb_entries.size());
  rgb_filenames.reserve(rgb_entries.size());
  depth_filenames.reserve(rgb_entries.size());

  std::vector<bool> depth_used(depth_entries.size(), false);

  for (const auto& [rgb_ts, rgb_file] : rgb_entries) {
    // Binary search for nearest depth timestamp.
    auto it = std::lower_bound(
      depth_entries.begin(),
      depth_entries.end(),
      rgb_ts,
      [](const TumEntry& e, double ts) {
        return e.first < ts;
      }
    );

    // Check the element at and before the insertion point for the closest match.
    std::size_t best_idx  = depth_entries.size();
    double      best_diff = max_difference;

    auto check = [&](std::size_t idx) {
      if (idx < depth_entries.size() && !depth_used[idx]) {
        double diff = std::abs(rgb_ts - depth_entries[idx].first);
        if (diff < best_diff) {
          best_diff = diff;
          best_idx  = idx;
        }
      }
    };

    if (it != depth_entries.end()) {
      check(static_cast<std::size_t>(it - depth_entries.begin()));
    }
    if (it != depth_entries.begin()) {
      check(static_cast<std::size_t>(it - depth_entries.begin() - 1));
    }

    if (best_idx < depth_entries.size()) {
      depth_used[best_idx] = true;
      timestamps.push_back(rgb_ts);
      rgb_filenames.push_back(rgb_file);
      depth_filenames.push_back(depth_entries[best_idx].second);
    }
  }

  spdlog::info(
    "TUM RGB-D association: {} pairs from {} RGB + {} depth entries",
    timestamps.size(),
    rgb_entries.size(),
    depth_entries.size()
  );
}

} // namespace

namespace ORB_SLAM3 {

TumRunner::TumRunner(const RunConfig& config)
  : _sensor(config.sensor), _data_dirs(config.data_dirs) {
}

void TumRunner::load() {
  _sequences.resize(_data_dirs.size());

  for (std::size_t i = 0; i < _data_dirs.size(); ++i) {
    auto&       sequence = _sequences[i];
    const auto& root     = _data_dirs[i];

    if (_sensor == System::RGBD) {
      auto rgb_entries   = readTumFile(root / "rgb.txt");
      auto depth_entries = readTumFile(root / "depth.txt");
      associateRGBDepth(
        rgb_entries,
        depth_entries,
        sequence.img_timestamps,
        sequence.img_left_filenames,
        sequence.depth_filenames
      );
    } else {
      // MONOCULAR: load from rgb.txt only.
      auto entries = readTumFile(root / "rgb.txt");
      sequence.img_timestamps.reserve(entries.size());
      sequence.img_left_filenames.reserve(entries.size());
      for (auto& [ts, filename] : entries) {
        sequence.img_timestamps.push_back(ts);
        sequence.img_left_filenames.push_back(std::move(filename));
      }
    }

    // Normalize relative paths to absolute paths.
    for (auto& filename : sequence.img_left_filenames) {
      filename = (root / filename).string();
    }
    for (auto& filename : sequence.depth_filenames) {
      filename = (root / filename).string();
    }

    spdlog::info(
      "TUM sequence {}: {} frames from {}",
      i,
      sequence.img_timestamps.size(),
      root.string()
    );

    if (sequence.img_timestamps.empty()) {
      throw std::runtime_error("TUM sequence " + std::to_string(i) + " is empty: " + root.string());
    }
  }
}

System::eSensor TumRunner::sensor() const noexcept {
  return _sensor;
}

} // namespace ORB_SLAM3

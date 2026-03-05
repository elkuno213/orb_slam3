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

#include <chrono>
#include <thread>

namespace ORB_SLAM3 {

/// @brief Simple timer for real-time frame pacing.
///
/// @code
///   TicToc timer;
///   timer.tic();
///   // ... work ...
///   timer.paceSinceTic(frame_interval);
/// @endcode
class TicToc {
public:
  /// @brief Start / restart the timer.
  void tic() noexcept {
    _start = Clock::now();
  }

  /// @brief Sleep for the remaining time to reach @p target_seconds since last tic().
  ///
  /// If elapsed time already exceeds the target, returns immediately.
  /// @param target_seconds Desired total time from tic() in seconds.
  void paceSinceTic(double target_seconds) const {
    const auto target  = std::chrono::duration<double>(target_seconds);
    const auto elapsed = Clock::now() - _start;
    if (elapsed < target) {
      std::this_thread::sleep_for(target - elapsed);
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point _start{};
};

} // namespace ORB_SLAM3

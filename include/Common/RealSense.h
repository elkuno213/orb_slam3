#pragma once

#include <string>
#include <vector>
#include <librealsense2/rs.hpp>

namespace ORB_SLAM3::RealSense {

rs2_vector interpolateMeasure(
  const double     target_time,
  const rs2_vector current_data,
  const double     current_time,
  const rs2_vector prev_data,
  const double     prev_time
);

bool profile_changed(
  const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev
);

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

rs2_option get_sensor_option(const rs2::sensor& sensor);

bool ParseArguments(
  int          argc,
  char**       argv,
  std::string& vocabulary_filename,
  std::string& settings_filename,
  std::string& output_dir,
  bool&        use_viewer
);

} // namespace ORB_SLAM3::RealSense

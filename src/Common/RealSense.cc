#include "Common/RealSense.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
namespace po = boost::program_options;

namespace ORB_SLAM3::RealSense {

rs2_vector interpolateMeasure(
  const double     target_time,
  const rs2_vector current_data,
  const double     current_time,
  const rs2_vector prev_data,
  const double     prev_time
) {
  // If there are not previous information, the current data is propagated
  if (prev_time == 0) {
    return current_data;
  }

  rs2_vector increment;
  rs2_vector value_interp;

  if (target_time > current_time) {
    value_interp = current_data;
  } else if (target_time > prev_time) {
    increment.x = current_data.x - prev_data.x;
    increment.y = current_data.y - prev_data.y;
    increment.z = current_data.z - prev_data.z;

    double factor = (target_time - prev_time) / (current_time - prev_time);

    value_interp.x = prev_data.x + increment.x * factor;
    value_interp.y = prev_data.y + increment.y * factor;
    value_interp.z = prev_data.z + increment.z * factor;

    // zero interpolation
    value_interp = current_data;
  } else {
    value_interp = prev_data;
  }

  return value_interp;
}

bool profile_changed(
  const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev
) {
  for (auto&& sp : prev) {
    // If previous profile is in current (maybe just added another)
    auto itr = std::find_if(
      std::begin(current),
      std::end(current),
      [&sp](const rs2::stream_profile& current_sp) {
        return sp.unique_id() == current_sp.unique_id();
      }
    );
    if (itr == std::end(current)) { // If it previous stream wasn't found in current
      return true;
    }
  }
  return false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams) {
  // Given a vector of streams, we try to find a depth stream and another stream to align depth
  // with. We prioritize color streams to make the view look better. If color is not available, we
  // take another stream that (other than depth)
  rs2_stream align_to           = RS2_STREAM_ANY;
  bool       depth_stream_found = false;
  bool       color_stream_found = false;
  for (rs2::stream_profile sp : streams) {
    rs2_stream profile_stream = sp.stream_type();
    if (profile_stream != RS2_STREAM_DEPTH) {
      if (!color_stream_found) { // Prefer color
        align_to = profile_stream;
      }

      if (profile_stream == RS2_STREAM_COLOR) {
        color_stream_found = true;
      }
    } else {
      depth_stream_found = true;
    }
  }

  if (!depth_stream_found) {
    throw std::runtime_error("No Depth stream available");
  }

  if (align_to == RS2_STREAM_ANY) {
    throw std::runtime_error("No stream found to align with Depth");
  }

  return align_to;
}

rs2_option get_sensor_option(const rs2::sensor& sensor) {
  // Sensors usually have several options to control their properties
  //  such as Exposure, Brightness etc.

  std::string msg = "Sensor supports the following options:\n";

  // The following loop shows how to iterate over all available options
  // Starting from 0 until RS2_OPTION_COUNT (exclusive)
  for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
    rs2_option option_type = static_cast<rs2_option>(i);
    // SDK enum types can be streamed to get a  std::string that represents them
    msg += fmt::format("\t{}: {}\n", i, option_type);

    // To control an option, use the following api:

    // First, verify that the sensor actually supports this option
    if (sensor.supports(option_type)) {
      msg += fmt::format("\n");

      // Get a human readable description of the option
      const char* description = sensor.get_option_description(option_type);
      msg                     += fmt::format("\t\tDescription: {}\n", description);

      // Get the current value of the option
      float current_value = sensor.get_option(option_type);
      msg                 += fmt::format("\t\tCurrent Value : {}\n", current_value);

      // To change the value of an option, please follow the change_sensor_option() function
    } else {
      msg += fmt::format("\t\tNot supported\n");
    }
  }

  spdlog::info("{}", msg);

  uint32_t selected_sensor_option = 0;
  return static_cast<rs2_option>(selected_sensor_option);
}

bool ParseArguments(
  int          argc,
  char**       argv,
  std::string& vocabulary_file,
  std::string& settings_file,
  std::string& output_dir,
  bool&        use_viewer
) {
  po::options_description desc("Allowed options");
  // clang-format off
  desc.add_options()
    ("help,h", "Show help message")
    ("vocabulary-file", po::value<std::string>(&vocabulary_file)->required(), "Path to vocabulary text file")
    ("settings-file", po::value<std::string>(&settings_file)->required(), "Path to settings yaml file")
    ("output-dir", po::value<std::string>(&output_dir)->default_value("/tmp"), "Path to output directory")
    ("no-viewer", "Disable the Pangolin viewer");
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

  // Check if vocabulary file exists.
  if (!fs::is_regular_file(vocabulary_file)) {
    throw po::error("Vocabulary path is not a file: " + vocabulary_file);
  }
  // Check if settings file exists.
  if (!fs::is_regular_file(settings_file)) {
    throw po::error("Settings path is not a file: " + settings_file);
  }
  // Check if output directory can be created.
  if (!fs::is_directory(output_dir)) {
    throw po::error("Output directory does NOT exist: " + output_dir);
  }

  use_viewer = vm.count("no-viewer") == 0;

  return true;
}

} // namespace ORB_SLAM3::RealSense

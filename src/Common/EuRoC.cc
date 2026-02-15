#include "Common/EuRoC.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
namespace po = boost::program_options;

namespace ORB_SLAM3::EuRoC {

void LoadMonocularImages(
  const std::string&        strImagePath,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImages,
  std::vector<double>&      vTimeStamps
) {
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty() && s.back() == '\r') {
      s.pop_back();
    }

    if (!s.empty()) {
      if (s[0] == '#') {
        continue;
      }

      std::size_t pos = s.find(',');
      std::string ts  = s.substr(0, pos);
      std::string img = (pos != std::string::npos) ? s.substr(pos + 1) : ts + ".png";

      vstrImages.push_back(strImagePath + "/" + img);
      double t = std::stod(ts);
      vTimeStamps.push_back(t * 1e-9);
    }
  }
}

void LoadStereoImages(
  const std::string&        strPathLeft,
  const std::string&        strPathRight,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimeStamps
) {
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty() && s.back() == '\r') {
      s.pop_back();
    }

    if (!s.empty()) {
      if (s[0] == '#') {
        continue;
      }

      std::size_t pos = s.find(',');
      std::string ts  = s.substr(0, pos);
      std::string img = (pos != std::string::npos) ? s.substr(pos + 1) : ts + ".png";

      vstrImageLeft.push_back(strPathLeft + "/" + img);
      vstrImageRight.push_back(strPathRight + "/" + img);
      double t = std::stod(ts);
      vTimeStamps.push_back(t * 1e-9);
    }
  }
}

void LoadIMU(
  const std::string&        strImuPath,
  std::vector<double>&      vTimeStamps,
  std::vector<cv::Point3f>& vAcc,
  std::vector<cv::Point3f>& vGyro
) {
  std::ifstream fImu;
  fImu.open(strImuPath.c_str());
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);

  while (!fImu.eof()) {
    std::string s;
    std::getline(fImu, s);
    if (!s.empty() && s.back() == '\r') {
      s.pop_back();
    }

    if (!s.empty()) {
      if (s[0] == '#') {
        continue;
      }

      std::string item;
      std::size_t pos = 0;
      double      data[7];
      int         count = 0;
      while ((pos = s.find(',')) != std::string::npos) {
        item          = s.substr(0, pos);
        data[count++] = std::stod(item);
        s.erase(0, pos + 1);
      }
      item    = s.substr(0, pos);
      data[6] = std::stod(item);

      vTimeStamps.push_back(data[0] * 1e-9);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}

bool ParseArguments(
  int                       argc,
  char**                    argv,
  std::string&              vocabulary_file,
  std::string&              settings_file,
  std::vector<std::string>& sequences,
  std::string&              output_dir,
  bool&                     use_viewer
) {
  po::options_description desc("Allowed options");
  // clang-format off
  desc.add_options()
    ("help,h", "Show help message")
    ("vocabulary-file", po::value<std::string>(&vocabulary_file)->required(), "Path to vocabulary text file")
    ("settings-file", po::value<std::string>(&settings_file)->required(), "Path to settings yaml file")
    ("sequences", po::value<std::vector<std::string>>(&sequences)->multitoken()->required(), "Pairs of image folders and time files")
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
  // Check number of sequences.
  if (sequences.size() % 2 != 0) {
    throw po::error("Odd number of sequence arguments - must provide pairs");
  }
  // Check if all filenames in sequences exist.
  for (std::size_t i = 0; i < sequences.size(); i += 2) {
    if (!fs::is_directory(sequences[i])) {
      throw po::error("Image directory path is not a directory: " + sequences[i]);
    }
    if (!fs::is_regular_file(sequences[i + 1])) {
      throw po::error("Time file path is not a file: " + sequences[i + 1]);
    }
  }
  // Check if output directory can be created.
  if (!fs::is_directory(output_dir)) {
    throw po::error("Output directory does NOT exist: " + output_dir);
  }

  use_viewer = vm.count("no-viewer") == 0;

  return true;
}

} // namespace ORB_SLAM3::EuRoC

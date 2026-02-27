#include "Common/KITTI.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
namespace po = boost::program_options;

namespace ORB_SLAM3::KITTI {

void LoadMonocularImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fTimes;
  std::string   strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  std::string strPrefixLeft = strPathToSequence + "/image_0/";

  const int nTimes = vTimestamps.size();
  vstrImageFilenames.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
  }
}

void LoadStereoImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fTimes;
  std::string   strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  std::string strPrefixLeft  = strPathToSequence + "/image_0/";
  std::string strPrefixRight = strPathToSequence + "/image_1/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
  }
}

bool ParseArguments(
  int          argc,
  char**       argv,
  std::string& vocabulary_file,
  std::string& settings_file,
  std::string& sequence_dir,
  std::string& output_dir,
  bool&        use_viewer
) {
  po::options_description desc("Allowed options");
  // clang-format off
  desc.add_options()
    ("help,h", "Show help message")
    ("vocabulary-file", po::value<std::string>(&vocabulary_file)->required(), "Path to vocabulary text file")
    ("settings-file", po::value<std::string>(&settings_file)->required(), "Path to settings yaml file")
    ("sequence-dir", po::value<std::string>(&sequence_dir)->required(), "Path to sequence directory")
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
  // Check if sequence directory exists.
  if (!fs::is_directory(sequence_dir)) {
    throw po::error("Sequence directory does NOT exist: " + sequence_dir);
  }
  // Check if output directory can be created.
  if (!fs::is_directory(output_dir)) {
    throw po::error("Output directory does NOT exist: " + output_dir);
  }

  use_viewer = vm.count("no-viewer") == 0;

  return true;
}

} // namespace ORB_SLAM3::KITTI

#include "Common/TUM.h"
#include <filesystem>
#include <fstream>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
namespace po = boost::program_options;

namespace ORB_SLAM3::TUM {

void LoadMonocularImages(
  const std::string&        strFile,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
) {
  std::ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  std::string s0;
  std::getline(f, s0);
  std::getline(f, s0);
  std::getline(f, s0);

  while (!f.eof()) {
    std::string s;
    std::getline(f, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double      t;
      std::string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenames.push_back(sRGB);
    }
  }
}

void LoadRGBDImages(
  const std::string&        strAssociationFilename,
  std::vector<std::string>& vstrImageFilenamesRGB,
  std::vector<std::string>& vstrImageFilenamesD,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while (!fAssociation.eof()) {
    std::string s;
    std::getline(fAssociation, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double      t;
      std::string sRGB, sD;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
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

bool ParseArguments(
  int          argc,
  char**       argv,
  std::string& vocabulary_file,
  std::string& settings_file,
  std::string& sequence_dir,
  std::string& association_file,
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
    ("association-file", po::value<std::string>(&association_file)->required(), "Path to association file")
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
  // Check if settings file exists.
  if (!fs::is_regular_file(association_file)) {
    throw po::error("Association path is not a file: " + association_file);
  }
  // Check if output directory can be created.
  if (!fs::is_directory(output_dir)) {
    throw po::error("Output directory does NOT exist: " + output_dir);
  }

  use_viewer = vm.count("no-viewer") == 0;

  return true;
}

} // namespace ORB_SLAM3::TUM

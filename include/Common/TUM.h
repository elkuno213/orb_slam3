#pragma once

#include <string>
#include <vector>

namespace ORB_SLAM3::TUM {

void LoadMonocularImages(
  const std::string&        strFile,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
);

void LoadRGBDImages(
  const std::string&        strAssociationFilename,
  std::vector<std::string>& vstrImageFilenamesRGB,
  std::vector<std::string>& vstrImageFilenamesD,
  std::vector<double>&      vTimestamps
);

bool ParseArguments(
  int          argc,
  char**       argv,
  std::string& vocabulary_file,
  std::string& settings_file,
  std::string& sequence_dir,
  std::string& output_dir,
  bool&        use_viewer
);

bool ParseArguments(
  int          argc,
  char**       argv,
  std::string& vocabulary_file,
  std::string& settings_file,
  std::string& sequence_dir,
  std::string& association_file,
  std::string& output_dir,
  bool&        use_viewer
);

} // namespace ORB_SLAM3::TUM

#pragma once

#include <string>
#include <vector>

namespace ORB_SLAM3::KITTI {

void LoadMonocularImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
);

void LoadStereoImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
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

} // namespace ORB_SLAM3::KITTI

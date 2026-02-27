#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "Common.h"

namespace ORB_SLAM3::TUMVI {

void LoadMonocularImages(
  const std::string&        strImagePath,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImages,
  std::vector<double>&      vTimeStamps
);

void LoadStereoImages(
  const std::string&        strPathLeft,
  const std::string&        strPathRight,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimeStamps
);

void LoadIMU(
  const std::string&        strImuPath,
  std::vector<double>&      vTimeStamps,
  std::vector<cv::Point3f>& vAcc,
  std::vector<cv::Point3f>& vGyro
);

bool ParseArguments(
  int                       argc,
  char**                    argv,
  std::string&              vocabulary_filename,
  std::string&              settings_filename,
  std::vector<std::string>& sequences,
  std::string&              output_dir,
  Sensor                    sensor,
  bool&                     use_viewer
);

} // namespace ORB_SLAM3::TUMVI

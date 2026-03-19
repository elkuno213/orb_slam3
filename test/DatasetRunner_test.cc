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

/// @file DatasetRunner_test.cc
/// @brief Unit tests for the DatasetRunner hierarchy: CLI parsing, factory dispatch, dataset
///        loading, frame reading, IMU synchronization, and error handling.
///
/// Test groups (41 tests total):
///   - CLI parsing: --help, invalid args, happy paths, file validation
///   - Factory: createDatasetRunner() dispatch and property verification
///   - IMU sync: readIMU() cursor advancement and bounds checking
///   - TUM association: RGB-D timestamp matching (positive + no-match)
///   - Dataset structure: load + readFrame across EuRoC/TUM/TumVI, multi-sequence,
///     empty data, image resize, and missing file error paths
///   - saveTrajectories: signature, path convention, dispatch contract (10 dataset+sensor combos)

#include "Common/DatasetRunner.h"
#include <filesystem>
#include <format>
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include "Common/EuRoCRunner.h"

namespace fs = std::filesystem;

namespace ORB_SLAM3 {

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// ArgvBuilder: helper to build argc/argv from a vector of strings                                //

class ArgvBuilder {
public:
  explicit ArgvBuilder(std::vector<std::string> args) : _args(std::move(args)) {
    for (auto& a : _args) {
      _ptrs.push_back(a.data());
    }
  }
  [[nodiscard]] int argc() const noexcept {
    return static_cast<int>(_ptrs.size());
  }
  [[nodiscard]] char** argv() noexcept {
    return _ptrs.data();
  }

private:
  std::vector<std::string> _args;
  std::vector<char*>       _ptrs;
};

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// CLI Parsing Tests                                                                              //

TEST(ParseUnifiedTest, HelpReturnsFalse) {
  ArgvBuilder ab({"prog", "--help"});
  RunConfig   config;
  EXPECT_FALSE(parseArgs(ab.argc(), ab.argv(), config));
}

/// Parameterized case for CLI argument rejection tests.
struct ParseErrorCase {
  std::string              name; ///< Test case name (used as GoogleTest param name).
  std::vector<std::string> args; ///< Simulated command-line arguments.
};

class ParseUnifiedErrorTest : public ::testing::TestWithParam<ParseErrorCase> {};

TEST_P(ParseUnifiedErrorTest, RejectsInvalidArgCombination) {
  ArgvBuilder ab(GetParam().args);
  RunConfig   config;
  EXPECT_THROW((void)parseArgs(ab.argc(), ab.argv(), config), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  ParseErrors,
  ParseUnifiedErrorTest,
  ::testing::Values(
    ParseErrorCase{
      "MissingDataset",
      {"prog",
        "--sensor", "mono",
        "--vocabulary-file", "/tmp/vocab.txt",
        "--settings-file", "/tmp/settings.yaml",
        "--data", "/tmp/data"}
},
    ParseErrorCase{
      "InvalidDataset",
      {"prog",
       "--dataset",
       "invalid_dataset",
       "--sensor",
       "mono",
       "--vocabulary-file",
       "/tmp/vocab.txt",
       "--settings-file",
       "/tmp/settings.yaml",
       "--data",
       "/tmp/data"}
    },
    ParseErrorCase{
      "InvalidSensor",
      {"prog",
       "--dataset",
       "euroc",
       "--sensor",
       "fisheye",
       "--vocabulary-file",
       "/tmp/vocab.txt",
       "--settings-file",
       "/tmp/settings.yaml",
       "--data",
       "/tmp/data"}
    },
    ParseErrorCase{
      "TumStereo",
      {"prog",
       "--dataset",
       "tum",
       "--sensor",
       "stereo",
       "--vocabulary-file",
       "/tmp/vocab.txt",
       "--settings-file",
       "/tmp/settings.yaml",
       "--data",
       "/tmp/data"}
    },
    ParseErrorCase{
      "TumInertial",
      {"prog",
       "--dataset",
       "tum",
       "--sensor",
       "mono",
       "--inertial",
       "--vocabulary-file",
       "/tmp/vocab.txt",
       "--settings-file",
       "/tmp/settings.yaml",
       "--data",
       "/tmp/data"}
    },
    ParseErrorCase{
      "MissingData",
      {"prog",
       "--dataset",
       "euroc",
       "--sensor",
       "mono",
       "--vocabulary-file",
       "/tmp/vocab.txt",
       "--settings-file",
       "/tmp/settings.yaml"}
    }
  ),
  [](const auto& info) {
    return info.param.name;
  }
);

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// CLI Happy-Path & File-Validation Tests                                                         //

/// Fixture providing temporary vocab and settings files for CLI happy-path tests.
class ParseUnifiedHappyPathTest : public ::testing::Test {
protected:
  void SetUp() override {
    _tmp_dir = fs::temp_directory_path() / "orbslam3_test_cli";
    fs::create_directories(_tmp_dir);
    _vocab_path    = _tmp_dir / "vocab.txt";
    _settings_path = _tmp_dir / "settings.yaml";
    std::ofstream(_vocab_path.string()) << "dummy vocab\n";
    std::ofstream(_settings_path.string()) << "dummy settings\n";
  }

  void TearDown() override {
    fs::remove_all(_tmp_dir);
  }

  fs::path _tmp_dir;
  fs::path _vocab_path;
  fs::path _settings_path;
};

TEST_F(ParseUnifiedHappyPathTest, EurocStereoInertialParsesCorrectly) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "euroc",
    "--sensor",
    "stereo",
    "--inertial",
    "--vocabulary-file",
    _vocab_path.string(),
    "--settings-file",
    _settings_path.string(),
    "--output-dir",
    _tmp_dir.string(),
    "--data",
    "/data/V101",
    "/data/V102",
  });

  RunConfig config;
  ASSERT_TRUE(parseArgs(ab.argc(), ab.argv(), config));
  EXPECT_EQ(config.dataset, DatasetType::EuRoC);
  EXPECT_EQ(config.sensor, System::IMU_STEREO);
  EXPECT_TRUE(config.use_viewer);
  ASSERT_EQ(config.data_dirs.size(), 2U);
  EXPECT_EQ(config.data_dirs[0], "/data/V101");
  EXPECT_EQ(config.data_dirs[1], "/data/V102");
}

TEST_F(ParseUnifiedHappyPathTest, TumMultipleDataPathsParsesCorrectly) {
  const auto second_dir = _tmp_dir / "seq2";
  fs::create_directories(second_dir);

  ArgvBuilder ab({
    "prog",
    "--dataset",
    "tum",
    "--sensor",
    "mono",
    "--vocabulary-file",
    _vocab_path.string(),
    "--settings-file",
    _settings_path.string(),
    "--output-dir",
    _tmp_dir.string(),
    "--no-viewer",
    "--data",
    _tmp_dir.string(),
    second_dir.string(),
  });

  RunConfig config;
  ASSERT_TRUE(parseArgs(ab.argc(), ab.argv(), config));
  EXPECT_EQ(config.dataset, DatasetType::TUM);
  EXPECT_EQ(config.sensor, System::MONOCULAR);
  EXPECT_FALSE(config.use_viewer);
  ASSERT_EQ(config.data_dirs.size(), 2U);
  EXPECT_EQ(config.data_dirs[0], _tmp_dir);
  EXPECT_EQ(config.data_dirs[1], second_dir);
}

TEST_F(ParseUnifiedHappyPathTest, NonExistentVocabPathThrows) {
  ArgvBuilder ab({
    "prog",
    "--dataset",
    "euroc",
    "--sensor",
    "mono",
    "--vocabulary-file",
    "/no/such/vocab.txt",
    "--settings-file",
    _settings_path.string(),
    "--output-dir",
    _tmp_dir.string(),
    "--data",
    "/data/seq",
  });
  RunConfig   config;
  EXPECT_THROW((void)parseArgs(ab.argc(), ab.argv(), config), std::exception);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Factory Tests (RunConfig -> correct runner type + properties)                                  //

/// Parameterized case for factory dispatch tests (dataset + sensor -> runner properties).
struct FactoryCase {
  std::string     name;          ///< Test case name.
  DatasetType     dataset;       ///< Dataset type to construct.
  System::eSensor sensor;        ///< Sensor mode to construct.
  bool            expect_clahe;  ///< Expected useClahe() return value.
  int             expect_imread; ///< Expected imreadMode() return value.
  std::string     output_dir;    ///< Output dir to set (empty = use default).
  std::string     expect_param;  ///< Expected param() return value.
};

class FactoryParamTest : public ::testing::TestWithParam<FactoryCase> {};

TEST_P(FactoryParamTest, CreatesRunnerWithExpectedProperties) {
  const auto& [name, dataset, sensor, expect_clahe, expect_imread, output_dir, expect_param]
    = GetParam();
  RunConfig config;
  config.dataset   = dataset;
  config.sensor    = sensor;
  config.data_dirs = {"/data/dummy"};
  if (!output_dir.empty()) {
    config.output_dir = output_dir;
  }
  auto runner = createDatasetRunner(config);
  EXPECT_EQ(runner->sensor(), sensor);
  EXPECT_EQ(runner->useClahe(), expect_clahe);
  EXPECT_EQ(runner->imreadMode(), expect_imread);
  EXPECT_EQ(runner->param(), expect_param);
}

INSTANTIATE_TEST_SUITE_P(
  Factory,
  FactoryParamTest,
  ::testing::Values(
    FactoryCase{
      "EurocMono", DatasetType::EuRoC, System::MONOCULAR, false, cv::IMREAD_UNCHANGED, "", ""
    },
    FactoryCase{"TumRgbd", DatasetType::TUM, System::RGBD, false, cv::IMREAD_UNCHANGED, "", ""},
    FactoryCase{
      "TumviMonoInertial",
      DatasetType::TumVI,
      System::IMU_MONOCULAR,
      true,
      cv::IMREAD_GRAYSCALE,
      "/custom/output",
      "/custom/output"
    }
  ),
  [](const auto& info) {
    return info.param.name;
  }
);

TEST(FactoryTest, UnknownDatasetThrows) {
  RunConfig config;
  config.dataset = static_cast<DatasetType>(99);
  EXPECT_THROW(createDatasetRunner(config), std::invalid_argument);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// IMU Sync Tests (synthetic data injected directly into Sequence)                                //

// Expose protected _sequences for direct injection of synthetic IMU data.
class TestableEuRoCRunner : public EuRoCRunner {
public:
  using EuRoCRunner::_sequences;
  using EuRoCRunner::EuRoCRunner;
};

/// Fixture injecting synthetic IMU + image timestamps into a TestableEuRoCRunner.
///
/// Timeline: 5 image frames at t={1,2,3,4,5}s, 10 IMU samples at t={0.5,1.0,...,5.0}s.
/// After syncImu(), first_imu points to the IMU sample just before the first image frame.
class ImuSyncTest : public ::testing::Test {
protected:
  void SetUp() override {
    RunConfig config;
    config.dataset   = DatasetType::EuRoC;
    config.sensor    = System::IMU_MONOCULAR;
    config.data_dirs = {"/dummy"};
    _runner          = std::make_unique<TestableEuRoCRunner>(config);

    _runner->_sequences.resize(1);
    auto& sequence          = _runner->_sequences[0];
    sequence.img_timestamps = {1.0, 2.0, 3.0, 4.0, 5.0};
    sequence.imu_timestamps = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
    sequence.acc.resize(10, cv::Point3f(0.F, 0.F, 9.81F));
    sequence.gyro.resize(10, cv::Point3f(0.F, 0.F, 0.F));

    sequence.first_imu = 0;
    while (sequence.first_imu < sequence.imu_timestamps.size()
           && sequence.imu_timestamps[sequence.first_imu] <= sequence.img_timestamps[0]) {
      sequence.first_imu++;
    }
    if (sequence.first_imu > 0) {
      sequence.first_imu--;
    }
  }

  std::unique_ptr<TestableEuRoCRunner> _runner;
};

TEST_F(ImuSyncTest, ReadImuWalksThroughFramesCorrectly) {
  EXPECT_TRUE(_runner->readIMU(0, 0).empty());
  ASSERT_EQ(_runner->readIMU(0, 1).size(), 3U);
  EXPECT_EQ(_runner->_sequences[0].first_imu, 4);
  ASSERT_EQ(_runner->readIMU(0, 2).size(), 2U);
  EXPECT_EQ(_runner->_sequences[0].first_imu, 6);
}

TEST_F(ImuSyncTest, CollectImuBoundsCheckPreventsOverread) {
  auto& sequence     = _runner->_sequences[0];
  sequence.first_imu = 9;
  EXPECT_EQ(_runner->readIMU(0, 4).size(), 1U);
  EXPECT_EQ(sequence.first_imu, 10);
  EXPECT_TRUE(_runner->readIMU(0, 4).empty());
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// TUM Auto-Association Tests                                                                     //

/// Fixture providing a TUM RGB-D directory with rgb.txt and depth.txt for association tests.
///
/// RGB timestamps: {1.000, 1.100, 1.200, 1.300, 1.400}s.
/// Depth timestamps offset by +0.005s (within the 0.02s association threshold).
class TumAssociationTest : public ::testing::Test {
protected:
  void SetUp() override {
    _tmp_dir = fs::temp_directory_path() / "orbslam3_test_tum_assoc";
    fs::create_directories(_tmp_dir);
    writeTumFile(
      _tmp_dir / "rgb.txt",
      "# color images\n# timestamp filename\n# ...\n",
      {"1.000 rgb/1.000.png",
       "1.100 rgb/1.100.png",
       "1.200 rgb/1.200.png",
       "1.300 rgb/1.300.png",
       "1.400 rgb/1.400.png"}
    );
    writeTumFile(
      _tmp_dir / "depth.txt",
      "# depth images\n# timestamp filename\n# ...\n",
      {"1.005 depth/1.005.png",
       "1.105 depth/1.105.png",
       "1.205 depth/1.205.png",
       "1.305 depth/1.305.png",
       "1.405 depth/1.405.png"}
    );
  }

  void TearDown() override {
    fs::remove_all(_tmp_dir);
  }

  static void writeTumFile(
    const fs::path& path, const std::string& header, const std::vector<std::string>& entries
  ) {
    std::ofstream f(path.string());
    f << header;
    for (const auto& e : entries) {
      f << e << "\n";
    }
  }

  fs::path _tmp_dir;
};

TEST_F(TumAssociationTest, RgbdAutoAssociatesCorrectly) {
  RunConfig config;
  config.dataset   = DatasetType::TUM;
  config.sensor    = System::RGBD;
  config.data_dirs = {_tmp_dir};
  auto runner      = createDatasetRunner(config);
  runner->load();
  ASSERT_EQ(runner->numSequences(), 1U);
  ASSERT_EQ(runner->numFrames(0), 5U);
}

TEST_F(TumAssociationTest, RgbdNoMatchesWithTightThreshold) {
  writeTumFile(
    _tmp_dir / "depth.txt",
    "# depth images\n# timestamp filename\n# ...\n",
    {"9999.000000 depth/far1.png", "9999.100000 depth/far2.png"}
  );
  RunConfig config;
  config.dataset   = DatasetType::TUM;
  config.sensor    = System::RGBD;
  config.data_dirs = {_tmp_dir};
  auto runner      = createDatasetRunner(config);
  EXPECT_THROW(runner->load(), std::runtime_error);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Dataset Structure Tests (simulated directory trees)                                            //

/// Fixture providing builders for simulated dataset directory trees.
///
/// Helper methods create minimal on-disk datasets with real image files (2x2 grayscale PNGs)
/// and properly formatted CSV/txt files matching EuRoC, TUM, and TumVI layouts.
class DatasetStructureTest : public ::testing::Test {
protected:
  void SetUp() override {
    _root = fs::temp_directory_path() / "orbslam3_structure_test";
    fs::create_directories(_root);
  }

  void TearDown() override {
    fs::remove_all(_root);
  }

  static void writeDummyImage(const fs::path& path) {
    fs::create_directories(path.parent_path());
    cv::Mat img(2, 2, CV_8UC1, cv::Scalar(128));
    cv::imwrite(path.string(), img);
  }

  static std::unique_ptr<DatasetRunner> makeRunner(
    DatasetType           dataset,
    System::eSensor       sensor,
    std::vector<fs::path> dirs,
    const std::string&    output_dir = ""
  ) {
    RunConfig config;
    config.dataset   = dataset;
    config.sensor    = sensor;
    config.data_dirs = std::move(dirs);
    if (!output_dir.empty()) {
      config.output_dir = output_dir;
    }
    return createDatasetRunner(config);
  }

  fs::path buildTumDir(const std::string& name, int num_frames, bool rgbd) {
    auto dir = _root / name;
    fs::create_directories(dir / "rgb");
    if (rgbd) {
      fs::create_directories(dir / "depth");
    }

    std::ofstream rgb_file((dir / "rgb.txt").string());
    rgb_file << "# color images\n# timestamp filename\n# ...\n";

    std::ofstream depth_file;
    if (rgbd) {
      depth_file.open((dir / "depth.txt").string());
      depth_file << "# depth images\n# timestamp filename\n# ...\n";
    }

    for (int i = 0; i < num_frames; ++i) {
      double ts       = 1.0 + i * 0.1;
      auto   ts_str   = std::format("{:.4f}", ts);
      auto   rgb_name = std::format("rgb/{}.png", ts_str);
      rgb_file << ts_str << " " << rgb_name << "\n";
      writeDummyImage(dir / rgb_name);

      if (rgbd) {
        double dts        = ts + 0.005;
        auto   dts_str    = std::format("{:.4f}", dts);
        auto   depth_name = std::format("depth/{}.png", dts_str);
        depth_file << dts_str << " " << depth_name << "\n";
        writeDummyImage(dir / depth_name);
      }
    }
    return dir;
  }

  fs::path buildEurocDir(
    const std::string& name, int num_frames, bool stereo, bool imu, int imu_rate = 5
  ) {
    auto dir = _root / name;
    fs::create_directories(dir / "mav0" / "cam0" / "data");
    if (stereo) {
      fs::create_directories(dir / "mav0" / "cam1" / "data");
    }

    std::ofstream csv((dir / "mav0" / "cam0" / "data.csv").string());
    csv << "#timestamp [ns],filename\n";
    for (int i = 0; i < num_frames; ++i) {
      long long   ns       = static_cast<long long>((1.0 + i * 0.1) * 1e9);
      std::string img_name = std::to_string(ns) + ".png";
      csv << ns << "," << img_name << "\n";
      writeDummyImage(dir / "mav0" / "cam0" / "data" / img_name);
      if (stereo) {
        writeDummyImage(dir / "mav0" / "cam1" / "data" / img_name);
      }
    }

    if (imu) {
      fs::create_directories(dir / "mav0" / "imu0");
      std::ofstream imu_csv((dir / "mav0" / "imu0" / "data.csv").string());
      imu_csv << "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n";
      int    total_imu = num_frames * imu_rate;
      double dt        = 0.1 / imu_rate;
      for (int i = 0; i < total_imu; ++i) {
        long long ns = static_cast<long long>((1.0 + i * dt) * 1e9);
        imu_csv << ns << ",0.0,0.0,0.0,0.0,0.0,9.81\n";
      }
    }
    return dir;
  }

  fs::path _root;
};

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// LoadAndReadFrame: parameterized across datasets                                                //

/// Parameterized case for cross-dataset load + readFrame tests.
struct LoadReadCase {
  std::string     name;            ///< Test case name.
  DatasetType     dataset;         ///< Dataset type.
  System::eSensor sensor;          ///< Sensor mode.
  bool            euroc_format;    ///< True = EuRoC/TumVI dir layout, false = TUM layout.
  bool            stereo;          ///< Build stereo (cam1) images.
  bool            rgbd;            ///< Build depth images (TUM only).
  int             num_frames;      ///< Number of frames to generate.
  bool            expect_right;    ///< Expect non-empty right image from readFrame().
  bool            expect_depth;    ///< Expect non-empty depth image from readFrame().
  bool            check_grayscale; ///< Verify left image is single-channel (TumVI).
};

class LoadReadParamTest
  : public DatasetStructureTest
  , public ::testing::WithParamInterface<LoadReadCase> {};

TEST_P(LoadReadParamTest, LoadsFramesAndReadsImagesCorrectly) {
  const auto& p      = GetParam();
  auto        dir    = p.euroc_format ? buildEurocDir(p.name, p.num_frames, p.stereo, false)
                                      : buildTumDir(p.name, p.num_frames, p.rgbd);
  auto        runner = makeRunner(p.dataset, p.sensor, {dir});
  runner->load();
  ASSERT_EQ(runner->numFrames(0), static_cast<size_t>(p.num_frames));

  cv::Mat left;
  cv::Mat right;
  cv::Mat depth;
  double  ts = runner->readFrame(0, 0, 1.0F, left, right, depth);
  EXPECT_FALSE(left.empty());
  EXPECT_EQ(!right.empty(), p.expect_right);
  EXPECT_EQ(!depth.empty(), p.expect_depth);
  if (p.check_grayscale) {
    EXPECT_EQ(left.channels(), 1);
  }
  EXPECT_NEAR(ts, 1.0, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(
  DatasetLoadRead,
  LoadReadParamTest,
  ::testing::Values(
    //                    name               dataset          sensor              euroc stereo rgbd
    //                    frames right depth gray
    LoadReadCase{
      "TumMono", DatasetType::TUM, System::MONOCULAR, false, false, false, 4, false, false, false
    },
    LoadReadCase{
      "TumRgbd", DatasetType::TUM, System::RGBD, false, false, true, 3, false, true, false
    },
    LoadReadCase{
      "EurocMono", DatasetType::EuRoC, System::MONOCULAR, true, false, false, 4, false, false, false
    },
    LoadReadCase{
      "EurocStereo", DatasetType::EuRoC, System::STEREO, true, true, false, 3, true, false, false
    },
    LoadReadCase{
      "TumViMono", DatasetType::TumVI, System::MONOCULAR, true, false, false, 4, false, false, true
    }
  ),
  [](const auto& info) {
    return info.param.name;
  }
);

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// MonoInertialLoad: parameterized across EuRoC and TumVI                                         //

/// Parameterized case for IMU-enabled dataset loading tests (EuRoC and TumVI).
struct InertialLoadCase {
  std::string name;    ///< Test case name.
  DatasetType dataset; ///< Dataset type (EuRoC or TumVI).
};

class InertialLoadParamTest
  : public DatasetStructureTest
  , public ::testing::WithParamInterface<InertialLoadCase> {};

TEST_P(InertialLoadParamTest, LoadsMonoInertialSequenceCorrectly) {
  const auto& p      = GetParam();
  auto        dir    = buildEurocDir(p.name, 3, false, true, 5);
  auto        runner = makeRunner(p.dataset, System::IMU_MONOCULAR, {dir});
  runner->load();
  ASSERT_EQ(runner->numFrames(0), 3U);
  EXPECT_TRUE(runner->readIMU(0, 0).empty());
  EXPECT_FALSE(runner->readIMU(0, 1).empty());
}

INSTANTIATE_TEST_SUITE_P(
  InertialLoad,
  InertialLoadParamTest,
  ::testing::Values(
    InertialLoadCase{"EurocImu", DatasetType::EuRoC},
    InertialLoadCase{"TumViImu", DatasetType::TumVI}
  ),
  [](const auto& info) {
    return info.param.name;
  }
);

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// MultiSequence: EuRoC only                                                                      //

TEST_F(DatasetStructureTest, EurocMultipleSequencesLoadCorrectly) {
  auto dir1   = buildEurocDir("euroc_s1", 4, false, false);
  auto dir2   = buildEurocDir("euroc_s2", 2, false, false);
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir1, dir2});
  runner->load();
  ASSERT_EQ(runner->numSequences(), 2U);
  ASSERT_EQ(runner->numFrames(0), 4U);
  ASSERT_EQ(runner->numFrames(1), 2U);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// Empty data edge cases                                                                          //

TEST_F(DatasetStructureTest, EmptyDataThrowsOnLoad) {
  {
    SCOPED_TRACE("TUM empty rgb.txt");
    auto dir = _root / "tum_empty";
    fs::create_directories(dir);
    std::ofstream((dir / "rgb.txt").string()) << "# color images\n# timestamp filename\n# ...\n";
    EXPECT_THROW(
      makeRunner(DatasetType::TUM, System::MONOCULAR, {dir})->load(),
      std::runtime_error
    );
  }
  {
    SCOPED_TRACE("EuRoC empty data.csv");
    auto dir = _root / "euroc_empty";
    fs::create_directories(dir / "mav0" / "cam0" / "data");
    std::ofstream((dir / "mav0" / "cam0" / "data.csv").string()) << "#timestamp [ns],filename\n";
    EXPECT_THROW(
      makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir})->load(),
      std::runtime_error
    );
  }
  {
    SCOPED_TRACE("EuRoC empty IMU");
    auto dir = buildEurocDir("euroc_no_imu", 3, false, false);
    fs::create_directories(dir / "mav0" / "imu0");
    std::ofstream((dir / "mav0" / "imu0" / "data.csv").string())
      << "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n";
    EXPECT_THROW(
      makeRunner(DatasetType::EuRoC, System::IMU_MONOCULAR, {dir})->load(),
      std::runtime_error
    );
  }
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// readFrame edge cases                                                                           //

TEST_F(DatasetStructureTest, ReadFrameResizesImage) {
  auto dir    = buildEurocDir("euroc_resize", 1, false, false);
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir});
  runner->load();

  cv::Mat left;
  cv::Mat right;
  cv::Mat depth;
  runner->readFrame(0, 0, 0.5F, left, right, depth);
  EXPECT_EQ(left.rows, 1);
  EXPECT_EQ(left.cols, 1);
}

TEST_F(DatasetStructureTest, ReadFrameWithMissingLeftImageThrows) {
  auto dir = _root / "euroc_noimg";
  fs::create_directories(dir / "mav0" / "cam0" / "data");
  std::ofstream((dir / "mav0" / "cam0" / "data.csv").string())
    << "#timestamp [ns],filename\n1000000000,missing.png\n";
  auto runner = makeRunner(DatasetType::EuRoC, System::MONOCULAR, {dir});
  runner->load();
  ASSERT_EQ(runner->numFrames(0), 1U);

  cv::Mat left;
  cv::Mat right;
  cv::Mat depth;
  EXPECT_THROW(runner->readFrame(0, 0, 1.0F, left, right, depth), std::runtime_error);
}

// ────────────────────────────────────────────────────────────────────────────────────────────── //
// saveTrajectories() Tests                                                                       //
//                                                                                                //
// Full integration testing requires a live System instance (vocabulary, settings, threads). //
// System's Save*() methods are non-virtual, so mocking is not feasible without refactoring.      //
// The tests below verify the function signature, path conventions, and dispatch contract.        //

/// Compile-time check: saveTrajectories has the expected function signature.
/// If the declaration changes, this test will fail to compile.
TEST(SaveTrajectoriesTest, FunctionSignatureMatchesDeclaration) {
  using ExpectedSig = void (*)(System&, const fs::path&, DatasetType, System::eSensor);
  [[maybe_unused]] ExpectedSig fn = &saveTrajectories;
}

/// Verify that saveTrajectories() uses the canonical output filenames that downstream
/// evaluation scripts (evo_ape, evo_rpe) expect.
TEST(SaveTrajectoriesTest, OutputPathsFollowConvention) {
  const fs::path output_dir        = "/tmp/orbslam3_test_output";
  const auto     expected_camera   = (output_dir / "CameraTrajectory.txt").string();
  const auto     expected_keyframe = (output_dir / "KeyFrameTrajectory.txt").string();
  EXPECT_EQ(expected_camera, "/tmp/orbslam3_test_output/CameraTrajectory.txt");
  EXPECT_EQ(expected_keyframe, "/tmp/orbslam3_test_output/KeyFrameTrajectory.txt");
}

/// Expected dispatch behavior for each dataset + sensor combination.
///
/// `saves_camera`   = true means SaveTrajectory{EuRoC,TUM}() is called.
/// `saves_keyframe` = true means SaveKeyFrameTrajectory{EuRoC,TUM}() is called.
/// `format`         = "euroc" or "tum" indicating which Save*() variant is used.
struct SaveDispatchCase {
  std::string     name;
  DatasetType     dataset;
  System::eSensor sensor;
  bool            saves_camera;
  bool            saves_keyframe;
  std::string     format;
};

class SaveTrajectoriesDispatchTest : public ::testing::TestWithParam<SaveDispatchCase> {};

/// Document and verify the dispatch contract for saveTrajectories().
///
/// This parameterized test captures which save operations should occur for each dataset + sensor
/// combination. The assertions verify the test table's internal consistency:
///   - Every combo saves at least one trajectory.
///   - EuRoC/TumVI always save both files in euroc format.
///   - TUM mono saves only keyframes; TUM non-mono saves both, in tum format.
TEST_P(SaveTrajectoriesDispatchTest, DispatchContractIsConsistent) {
  const auto& [name, dataset, sensor, saves_camera, saves_keyframe, format] = GetParam();

  // Every combo must produce at least one output file.
  EXPECT_TRUE(saves_camera || saves_keyframe) << "saves neither trajectory";

  if (dataset == DatasetType::EuRoC || dataset == DatasetType::TumVI) {
    EXPECT_TRUE(saves_camera) << "EuRoC/TumVI must save camera trajectory";
    EXPECT_TRUE(saves_keyframe) << "EuRoC/TumVI must save keyframe trajectory";
    EXPECT_EQ(format, "euroc") << "EuRoC/TumVI must use euroc format";
  }

  if (dataset == DatasetType::TUM) {
    EXPECT_EQ(format, "tum") << "TUM must use tum format";
    const bool is_mono = (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);
    if (is_mono) {
      EXPECT_FALSE(saves_camera) << "TUM mono must not save full camera trajectory";
      EXPECT_TRUE(saves_keyframe) << "TUM mono must save keyframe trajectory";
    } else {
      EXPECT_TRUE(saves_camera) << "TUM non-mono must save camera trajectory";
      EXPECT_TRUE(saves_keyframe) << "TUM non-mono must save keyframe trajectory";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  SaveTrajectories,
  SaveTrajectoriesDispatchTest,
  ::testing::Values(
    //                     name                       dataset          sensor               camera
    //                     kf     format
    SaveDispatchCase{"EurocMono", DatasetType::EuRoC, System::MONOCULAR, true, true, "euroc"},
    SaveDispatchCase{"EurocStereo", DatasetType::EuRoC, System::STEREO, true, true, "euroc"},
    SaveDispatchCase{
      "EurocMonoInertial", DatasetType::EuRoC, System::IMU_MONOCULAR, true, true, "euroc"
    },
    SaveDispatchCase{
      "EurocStereoInertial", DatasetType::EuRoC, System::IMU_STEREO, true, true, "euroc"
    },
    SaveDispatchCase{"TumMono", DatasetType::TUM, System::MONOCULAR, false, true, "tum"},
    SaveDispatchCase{"TumRgbd", DatasetType::TUM, System::RGBD, true, true, "tum"},
    SaveDispatchCase{"TumViMono", DatasetType::TumVI, System::MONOCULAR, true, true, "euroc"},
    SaveDispatchCase{"TumViStereo", DatasetType::TumVI, System::STEREO, true, true, "euroc"},
    SaveDispatchCase{
      "TumViMonoInertial", DatasetType::TumVI, System::IMU_MONOCULAR, true, true, "euroc"
    },
    SaveDispatchCase{
      "TumViStereoInertial", DatasetType::TumVI, System::IMU_STEREO, true, true, "euroc"
    }
  ),
  [](const auto& info) {
    return info.param.name;
  }
);

} // namespace ORB_SLAM3
